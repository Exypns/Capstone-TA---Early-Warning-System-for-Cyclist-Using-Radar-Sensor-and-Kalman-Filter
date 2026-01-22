#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// --- PIN CONFIG ---
#define BUZZER_PIN 14
#define VIBRO_PIN  21
#define LED_WARNING_PIN 4   // Waspada
#define LED_DANGER_PIN 2    // Bahaya

// --- BUZZER (LEDC) CONFIG ---
#define BUZZER_CHANNEL 0
#define BUZZER_FREQ    2000
#define BUZZER_RES     8    // 0–255 duty

// --- DATA STRUCT ---
typedef struct struct_message {
  float distance;
  float speed;
  uint8_t hazardLevel;
} struct_message;

struct_message incomingData;

// --- LOGIC LEVELS ---
#define LEVEL_SAFE    0
#define LEVEL_WARNING 1
#define LEVEL_DANGER  2

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 300; 
unsigned long lastRecvTime = 0;
const unsigned long SIGNAL_TIMEOUT = 2000;

unsigned long patternTimer = 0;
bool toggleState = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    lastRecvTime = millis();
}

void playNotification() {
    unsigned long now = millis();
    int interval = 0;
    int pwmValue = 0;

    // 1. Cek Signal Timeout
    if (now - lastRecvTime > SIGNAL_TIMEOUT) {
        ledcWrite(BUZZER_CHANNEL, 0);
        digitalWrite(LED_WARNING_PIN, LOW);
        digitalWrite(LED_DANGER_PIN, LOW);
        digitalWrite(VIBRO_PIN, LOW);
        return;
    }

    // 2. Tentukan Parameter berdasarkan Level Bahaya
    if (incomingData.hazardLevel == LEVEL_DANGER) {
        interval = 100; // Blink cepat (Intens)
        pwmValue = 255; // PWM Maksimal untuk Danger
    } 
    else if (incomingData.hazardLevel == LEVEL_WARNING) {
        interval = 500; // Blink lambat (Bip-Bip)
        pwmValue = 230; // PWM 230 sesuai permintaan
    } 
    else {
        // LEVEL_SAFE
        ledcWrite(BUZZER_CHANNEL, 0);
        digitalWrite(LED_WARNING_PIN, LOW);
        digitalWrite(LED_DANGER_PIN, LOW);
        digitalWrite(VIBRO_PIN, LOW);
        return;
    }

    // 3. Non-blocking Pattern (Logic Blink)
    if (now - patternTimer >= interval) {
        patternTimer = now;
        toggleState = !toggleState;
    }

    // 4. Eksekusi Output
    if (toggleState) {
        ledcWrite(BUZZER_CHANNEL, pwmValue);
        digitalWrite(VIBRO_PIN, HIGH);
        // Nyalakan LED sesuai level
        if (incomingData.hazardLevel == LEVEL_DANGER) {
            digitalWrite(LED_DANGER_PIN, HIGH);
            digitalWrite(LED_WARNING_PIN, LOW);
        } else {
            digitalWrite(LED_WARNING_PIN, HIGH);
            digitalWrite(LED_DANGER_PIN, LOW);
        }
    } else {
        ledcWrite(BUZZER_CHANNEL, 0);
        digitalWrite(VIBRO_PIN, LOW);
        digitalWrite(LED_WARNING_PIN, LOW);
        digitalWrite(LED_DANGER_PIN, LOW);
    }
}

void printReceivedData() {

    unsigned long now = millis();
    if (now - lastPrintTime < PRINT_INTERVAL) return;
    lastPrintTime = now;

    if (now - lastRecvTime > SIGNAL_TIMEOUT) {
        Serial.println("[NO SIGNAL]");
        return;
    }
    Serial.print("Distance: ");
    Serial.print(incomingData.distance, 2);
    Serial.print(" m | ");

    Serial.print("Speed: ");
    Serial.print(incomingData.speed, 2);
    Serial.print(" km/h | ");

    Serial.print("Level: ");
    switch (incomingData.hazardLevel) {
        case LEVEL_SAFE:
            Serial.println("SAFE");
            break;
        case LEVEL_WARNING:
            Serial.println("WARNING");
            break;
        case LEVEL_DANGER:
            Serial.println("DANGER");
            break;
        default:
            Serial.println("UNKNOWN");
            break;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(VIBRO_PIN, OUTPUT);
    pinMode(LED_WARNING_PIN, OUTPUT);
    pinMode(LED_DANGER_PIN, OUTPUT);

    ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    playNotification();
    printReceivedData();
}
