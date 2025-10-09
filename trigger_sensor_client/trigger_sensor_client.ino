/*
  trigger_sensor_client.ino
  Halloween Skull Activation System – Remote motion sensor node.

  Responsibilities:
  - Monitor an HC-SR04 ultrasonic sensor for motion/close range.
  - When a visitor is detected, send an HTTP POST /activate request to the controller node.

  Hardware wiring (UNO R4 WiFi):
    HC-SR04 TRIG -> TRIG_PIN (defaults to D6)
    HC-SR04 ECHO -> ECHO_PIN (defaults to D7) with level shifting if required
    VCC/GND      -> 5V / GND
*/

#include <WiFiS3.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "No secrets.h found; using placeholder WiFi credentials"
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";
static const char CONTROLLER_HOST[] = "192.168.1.100";
#endif

// ==== WiFi credentials =======================================================

// ==== Controller endpoint ====================================================
const uint16_t CONTROLLER_PORT = 80;
const unsigned long RETRIGGER_DELAY_MS = 15000; // Don't retrigger until this cooldown expires
const unsigned long ACTIVATION_DURATION_MS = 8000;

// ==== Sensor configuration ===================================================
const uint8_t TRIG_PIN = 6;
const uint8_t ECHO_PIN = 7;
const float DETECTION_DISTANCE_CM = 200.0f;     // Trigger when closer than this
const float MIN_VALID_DISTANCE_CM = 2.0f;
const float MAX_VALID_DISTANCE_CM = 400.0f;
const uint8_t DETECTION_CONSISTENCY_COUNT = 3;  // Require N hits before triggering
const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 120;

unsigned long lastSampleAt = 0;
unsigned long lastTriggerAt = 0;
uint8_t detectionHits = 0;
float lastDistanceCm = MAX_VALID_DISTANCE_CM;

void ensureWifiConnected();
float readDistanceCm();
bool triggerController();
void logWifiStatus();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for Serial
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ensureWifiConnected();
  logWifiStatus();
}

void loop() {
  ensureWifiConnected();

  const unsigned long now = millis();
  if (now - lastSampleAt < SENSOR_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleAt = now;

  const float distance = readDistanceCm();
  if (distance > 0) {
    lastDistanceCm = distance;
  }

  const bool withinRange = distance > MIN_VALID_DISTANCE_CM &&
                           distance <= DETECTION_DISTANCE_CM;

  if (withinRange) {
    if (detectionHits < 255) {
      detectionHits++;
    }
  } else {
    if (detectionHits > 0) {
      detectionHits--;
    }
  }

  if (detectionHits >= DETECTION_CONSISTENCY_COUNT &&
      (now - lastTriggerAt) >= RETRIGGER_DELAY_MS) {
    Serial.print("Visitor detected at ~");
    Serial.print(lastDistanceCm);
    Serial.println(" cm");

    if (triggerController()) {
      lastTriggerAt = now;
    } else {
      Serial.println("Controller trigger failed; will retry on next detection");
    }
    detectionHits = 0;
  }
}

// ==== WiFi helpers ===========================================================
void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) {
    Serial.println("  Connection failed, retrying in 2 seconds...");
    delay(2000);
  }

  Serial.println("WiFi connected");
  logWifiStatus();
}

void logWifiStatus() {
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

// ==== Controller trigger =====================================================
bool triggerController() {
  WiFiClient client;
  if (!client.connect(CONTROLLER_HOST, CONTROLLER_PORT)) {
    Serial.println("Failed to connect to controller host");
    return false;
  }

  char payload[32];
  snprintf(payload, sizeof(payload), "duration=%lu", ACTIVATION_DURATION_MS);

  client.print("POST /activate HTTP/1.1\r\n");
  client.print("Host: ");
  client.print(CONTROLLER_HOST);
  client.print("\r\n");
  client.println("Content-Type: application/x-www-form-urlencoded");
  client.print("Content-Length: ");
  client.print(strlen(payload));
  client.print("\r\n");
  client.println("Connection: close");
  client.println();
  client.print(payload);

  unsigned long startWait = millis();
  while (client.connected() && !client.available() && millis() - startWait < 2000) {
    delay(10);
  }

  bool success = false;
  while (client.available()) {
    String line = client.readStringUntil('\n');
    if (line.startsWith("HTTP/1.1")) {
      success = line.indexOf("200") > 0 || line.indexOf("202") > 0;
    }
  }

  client.stop();
  return success;
}

// ==== Sensor helpers ==========================================================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) {
    return -1.0f;
  }

  float distance = (duration / 2.0f) * 0.0343f;
  if (distance < MIN_VALID_DISTANCE_CM || distance > MAX_VALID_DISTANCE_CM) {
    return -1.0f;
  }
  return distance;
}
