#include <Arduino.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ---- PIN & BAUD ----
#define RX_LD50 9
#define TX_LD50 10
#define LD2450_BAUD 256000

#define RX_LD51 16
#define TX_LD51 17
#define LD2451_BAUD 115200

uint8_t receiverMacAddress[] = {0x1C, 0xDB, 0xD4, 0x3D, 0x1E, 0x68};

// --- KALMAN PARAMETER ---
float KS_P = 5.0;
float KS_R = 1.0;
float KS_Q = 5.0;

float KD_P = 5.0;
float KD_R = 1.0;
float KD_Q = 1.0;

// --- THRESHOLD ---
#define THRESHOLD_SPEED_DANGER  46.0
#define THRESHOLD_SPEED_WARNING 32.0 
#define THRESHOLD_DIST_DANGER   1.5 + 0.4 
#define THRESHOLD_DIST_WARNING  3.0 + 0.4

#define THRESHOLD_MIN_MOVEMENT 4.0

// --- DATA STRUCTURE ---
typedef struct  struct_message {
    float distance;
    float speed;
    uint8_t hazardLevel;    
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- KALMAN FILTER CLASS --
class KalmanFilterConfig {
    private:
        float _measurementError, _estimateError, _processNoiseFactor, 
        _currentEstimate, _previousEstimate, _kalmanGain;

    public:
        KalmanFilterConfig(
            float measurementError, 
            float initialEstimateError,
            float processNoiseFactor
        ) {
            _measurementError = measurementError;
            _estimateError = initialEstimateError;
            _processNoiseFactor = processNoiseFactor;

            _currentEstimate = 0.0;
            _previousEstimate = 0.0;
        }

    float updateEstimate(float measurement) {
        _kalmanGain = _estimateError / (_estimateError + _measurementError);
        _currentEstimate = _previousEstimate + _kalmanGain * 
        (measurement - _previousEstimate);
        _estimateError = (1.0 - _kalmanGain) * _estimateError + 
        fabs(_previousEstimate - _currentEstimate) * _processNoiseFactor;

        _previousEstimate = _currentEstimate;
        return _currentEstimate;
    }

    void setEstimate(float estimate) {
        _currentEstimate = estimate;
        _previousEstimate = estimate;
    }
};

// --- LD2450 VARIABLES ---
struct TargetLD50 {
    int16_t rawX, rawY, rawSpeed; // Raw Data
    int16_t filteredX, filteredY, filteredSpeed; // Kalman Filtered Data

    int count;

    float rawDistance, filteredDistance;    
    float rawSpeedKmh, filteredSpeedKmh;
};

TargetLD50 targetsLD50[3];
uint8_t bufferLD50[24];
uint8_t indexLD50 = 0;

enum ParserStateLD50 {
    WAIT_HEADER1, WAIT_HEADER2, WAIT_HEADER3, WAIT_HEADER4,
    READ_DATA50, 
    READ_TAIL1, READ_TAIL2
};

ParserStateLD50 parserStateLD50 = WAIT_HEADER1;

const uint8_t HEADER50[4] = { 0xAA, 0xFF, 0x03, 0x00 };
const uint8_t TAIL50[2] = { 0x55, 0xCC };

KalmanFilterConfig kfLD50_X[3] = { {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q} };
KalmanFilterConfig kfLD50_Y[3] = { {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q} };
KalmanFilterConfig kfLD50_Speed[3] = { {KS_R, KS_P, KS_Q}, {KS_R, KS_P, KS_Q}, {KS_R, KS_P, KS_Q} };

// --- LD2451 VARIABLES ---
struct TargetLD51 {
    float rawDistance, filteredDistance;
    float rawSpeed, filteredSpeed;

    uint8_t count;
};

TargetLD51 targetsLD51[3];
uint8_t bufferLD51[64];
uint16_t payloadLengthLD51 = 0;
uint16_t indexLD51 = 0;

enum ParserStateLD51 {
    WAIT_H1, WAIT_H2, WAIT_H3, WAIT_H4,
    READ_LEN_L, READ_LEN_H,
    READ_DATA51,
    READ_T1, READ_T2
};

ParserStateLD51 parserStateLD51 = WAIT_H1;

const uint8_t HEADER51[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
const uint8_t TAIL51[4] = { };

KalmanFilterConfig kfLD51_Distance[3] = { {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q}, {KD_R, KD_P, KD_Q} };
KalmanFilterConfig kfLD51_Speed[3] = { {KS_R, KS_P, KS_Q}, {KS_R, KS_P, KS_Q}, {KS_R, KS_P, KS_Q} };

unsigned long lastUpdateLD50 = 0;
unsigned long lastUpdateLD51 = 0;
const unsigned long DATA_TIMEOUT = 500;

// --- PARSING LD2450 ---
int16_t transformReading(uint16_t rawData) {
    bool isNegative = (rawData & 0x8000) != 0;
    int16_t value = rawData & 0x7FFF;
    return isNegative ? -value : value;
};

void parseDataLD50() {
    lastUpdateLD50 = millis();

    for (int i = 0; i < 3; i++) {
        int offset = i * 8;

        uint16_t rawX16 = (uint16_t)((bufferLD50[offset + 1] << 8) | bufferLD50[offset]);
        uint16_t rawY16 = (uint16_t)((bufferLD50[offset + 3] << 8) | bufferLD50[offset + 2]);
        uint16_t rawSpeed16 = (uint16_t)((bufferLD50[offset + 5] <<8) | bufferLD50[offset + 4]);

        int16_t rawX = transformReading(rawX16);
        int16_t rawY = (rawY16 & 0x8000) ? (rawY16 & 0x7FFF) : rawY16;
        int16_t rawSpeed = transformReading(rawSpeed16);

        if (rawY > 0) {
            if (targetsLD50[i].count == 0) {
                kfLD50_X[i].setEstimate(rawX);
                kfLD50_Y[i].setEstimate(rawY);
                kfLD50_Speed[i].setEstimate(rawSpeed);
            }
            if (targetsLD50[i].count < 20) targetsLD50[i].count++;

            targetsLD50[i].rawX = rawX;
            targetsLD50[i].rawY = rawY;
            targetsLD50[i].rawSpeed = rawSpeed;

            targetsLD50[i].filteredX = (int16_t)kfLD50_X[i].updateEstimate(rawX);
            targetsLD50[i].filteredY = (int16_t)kfLD50_Y[i].updateEstimate(rawY);
            targetsLD50[i].filteredSpeed = (int16_t)kfLD50_Speed[i].updateEstimate(rawSpeed);

            targetsLD50[i].rawDistance = sqrt((rawX * rawX) + (rawY * rawY)) / 1000.0;
            targetsLD50[i].filteredDistance = 
                sqrt((targetsLD50[i].filteredX * targetsLD50[i].filteredX) + 
                (targetsLD50[i].filteredY * targetsLD50[i].filteredY)) / 1000.0;

            targetsLD50[i].rawSpeedKmh = abs(rawSpeed) * 0.036;
            targetsLD50[i].filteredSpeedKmh = abs(targetsLD50[i].filteredSpeed) * 0.036;

        } else {
            if (targetsLD50[i].count > 0) targetsLD50[i].count -= 2;
            if (targetsLD50[i].count < 0) targetsLD50[i].count = 0;

            if (targetsLD50[i].count == 0) {
                targetsLD50[i].rawSpeed = 0;
                targetsLD50[i].filteredSpeed = 0;
                targetsLD50[i].rawDistance = 0;
                targetsLD50[i].filteredDistance = 0;
            }
        }
    }
}

void processLD50DataFrame() {
    while (Serial1.available()) {
        uint8_t byteIn = Serial1.read();
        switch (parserStateLD50) {
        case WAIT_HEADER1: 
            if (byteIn == HEADER50[0]) 
                parserStateLD50 = WAIT_HEADER2;
            break;
        case WAIT_HEADER2:
            if (byteIn == HEADER50[1])
                parserStateLD50 = WAIT_HEADER3;
            else 
                parserStateLD50 = WAIT_HEADER1;
            break;
        case WAIT_HEADER3:
            if (byteIn == HEADER50[2])
                parserStateLD50 = WAIT_HEADER4;
            else 
                parserStateLD50 = WAIT_HEADER1;
            break;
        case WAIT_HEADER4:
            if (byteIn == HEADER50[3]) {
                parserStateLD50 = READ_DATA50;
                indexLD50 = 0;
            } else 
                parserStateLD50 = WAIT_HEADER1;
            break;
        case READ_DATA50:
            if (indexLD50 < sizeof(bufferLD50))
                bufferLD50[indexLD50++] = byteIn;
            if (indexLD50 >= sizeof(bufferLD50))
                parserStateLD50 = READ_TAIL1;
            break;
        case READ_TAIL1: 
            if (byteIn == TAIL50[0])
                parserStateLD50 = READ_TAIL2;
            else 
                parserStateLD50 = WAIT_HEADER1;
            break;
        case READ_TAIL2:
            if (byteIn == TAIL50[1]) {
                parseDataLD50(); }
            parserStateLD50 = WAIT_HEADER1;
            break;
        }
    }
}

// --- PARSING LD2451 ---
void parseDataLD51(const uint8_t *payload, uint16_t length) {
    if (length < 7) return;

    uint8_t targetCount = payload[0];
    if (targetCount > 3)
        targetCount = 3;

    if (targetCount == 0) {
        for (int i = 0; i < 3; i++) targetsLD51[i].count = 0;
        return;
    }

    for (uint8_t i = 0; i < targetCount; i++) {
        int baseIndex = 2 + (i * 5);

        float rawDistance = (float)payload[baseIndex + 1];
        uint8_t direction = payload[baseIndex + 2];
        float rawSpeed    = (float)payload[baseIndex + 3];

        targetsLD51[i].rawDistance = rawDistance;
        targetsLD51[i].rawSpeed    = rawSpeed;
        
        if (targetsLD51[i].count == 0) {
            kfLD51_Distance[i].setEstimate(rawDistance);
            kfLD51_Speed[i].setEstimate(rawSpeed);
        }

        if (targetsLD51[i].count < 20)
            targetsLD51[i].count++;

        targetsLD51[i].filteredDistance = kfLD51_Distance[i].updateEstimate(rawDistance);
        targetsLD51[i].filteredSpeed = kfLD51_Speed[i].updateEstimate(rawSpeed);
    }
}

void processLD51DataFrame() {
     while (Serial2.available()) {
        uint8_t incomingByte = Serial2.read();
        switch (parserStateLD51) {
        case WAIT_H1: if (incomingByte == HEADER51[0]) parserStateLD51 = WAIT_H2; break;
        case WAIT_H2: if (incomingByte == HEADER51[1]) parserStateLD51 = WAIT_H3; 
                        else parserStateLD51 = WAIT_H1; break;
        case WAIT_H3: if (incomingByte == HEADER51[2]) parserStateLD51 = WAIT_H4;            
                        else parserStateLD51 = WAIT_H1; break;
        case WAIT_H4: if (incomingByte == HEADER51[3]) parserStateLD51 = READ_LEN_L;
                        else parserStateLD51 = WAIT_H1; break;
        case READ_LEN_L: payloadLengthLD51 = incomingByte; parserStateLD51 = READ_LEN_H;
            break;
        case READ_LEN_H:
            payloadLengthLD51 |= (incomingByte << 8);
            if (payloadLengthLD51 == 0 || payloadLengthLD51 > sizeof(bufferLD51)) {
                parserStateLD51 = WAIT_H1; break; }
            indexLD51 = 0; parserStateLD51 = READ_DATA51;
            break;
        case READ_DATA51:
            bufferLD51[indexLD51++] = incomingByte;
            if (indexLD51 >= payloadLengthLD51) {
                parseDataLD51(bufferLD51, payloadLengthLD51);
                lastUpdateLD51 = millis();
                parserStateLD51 = WAIT_H1;
            }
            break;
        }
    }
}

// --- MAIN LOGIC ---
unsigned long lastStateChange = 0;
const unsigned long HOLD_TIME = 1000; 

void sendAndPrint() {
    unsigned long now = millis();

    bool hasTarget50 = (now - lastUpdateLD50 <= DATA_TIMEOUT);
    bool hasTarget51 = (now - lastUpdateLD51 <= DATA_TIMEOUT);

    float finalDist = 99.0;
    float finalSpeed = 0.0;
    bool anyTarget = false;

    float minDist50 = 999.0, speed50 = 0.0;
    float minDist51 = 999.0, speed51 = 0.0;
    bool found50 = false, found51 = false;

    if (hasTarget50) {
        for (int i = 0; i < 3; i++) {
            if (targetsLD50[i].count > 5 && targetsLD50[i].filteredDistance < minDist50) {
                minDist50 = targetsLD50[i].filteredDistance;
                speed50 = targetsLD50[i].filteredSpeedKmh;
                found50 = true;
            }
        }
    }

    // Ambil data terdekat dari LD2451
    if (hasTarget51) {
        for (int i = 0; i < 3; i++) {
            if (targetsLD51[i].count > 0 && targetsLD51[i].filteredDistance < minDist51) {
                minDist51 = targetsLD51[i].filteredDistance;
                speed51 = abs(targetsLD51[i].filteredSpeed);
                found51 = true;
            }
        }
    }

    if (found50 && minDist50 <= THRESHOLD_DIST_DANGER) {
        finalDist = minDist50;
        finalSpeed = (found51 && speed51 > speed50) ? speed51 : speed50;
        anyTarget = true;
    } 
    else if (found51) {
        finalDist = minDist51;
        finalSpeed = (found50 && speed50 > speed51) ? speed50 : speed51;
        anyTarget = true;
    }
    else if (found50) {
        finalDist = minDist50;
        finalSpeed = speed50;
        anyTarget = true;
    }

    uint8_t currentRawStatus = 0; 
    if (anyTarget) {
        bool moving = (finalSpeed > THRESHOLD_MIN_MOVEMENT);
        if ((finalDist < THRESHOLD_DIST_DANGER && moving) || finalSpeed > THRESHOLD_SPEED_DANGER) {
            currentRawStatus = 2; // BAHAYA
        } 
        else if ((finalDist < THRESHOLD_DIST_WARNING && moving) || finalSpeed > THRESHOLD_SPEED_WARNING) {
            currentRawStatus = 1; // WASPADA
        }
    }

    static uint8_t filteredStatus = 0;
    if (currentRawStatus > filteredStatus) {
        filteredStatus = currentRawStatus;
        lastStateChange = now;
    } else if (currentRawStatus < filteredStatus) {
        if (now - lastStateChange > HOLD_TIME) {
            filteredStatus = currentRawStatus;
            lastStateChange = now;
        }
    } else {
        lastStateChange = now;
    }

    myData.distance = finalDist;
    myData.speed = finalSpeed;
    myData.hazardLevel = filteredStatus;
    esp_now_send(receiverMacAddress, (uint8_t*)&myData, sizeof(myData));

    Serial.printf("Mode: %s | Dist: %.2f | Spd: %.2f | Stat: %d\n",
                  (found50 && minDist50 <= THRESHOLD_DIST_DANGER) ? "URGENT-50" : (found51 ? "PRIO-51" : "STANDBY"),
                  finalDist, finalSpeed, filteredStatus);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

// --- SETUP ---
void setup() {
    Serial.begin(115200);

    Serial1.begin(LD2450_BAUD, SERIAL_8N1, RX_LD50, TX_LD50);
    Serial2.begin(LD2451_BAUD, SERIAL_8N1, RX_LD51, TX_LD51);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Init Failed"); return; }
    esp_now_register_send_cb(OnDataSent);
    
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    // esp_now_add_peer(&peerInfo);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer"); return; }
}

void loop() {
    processLD50DataFrame();
    processLD51DataFrame();

    static unsigned long lastLogic = 0;
    if (millis() - lastLogic > 50) {
        lastLogic = millis();
        sendAndPrint();
    }
}




