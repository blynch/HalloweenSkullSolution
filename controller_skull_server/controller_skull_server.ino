/*
  controller_skull_server.ino
  Halloween Skull Activation System – Controller node.

  Responsibilities:
  - Connect to WiFi and expose a simple HTTP API on port 80.
  - Toggle a relay that powers the skull props when /activate is called.
  - Read an HC-SR04 ultrasonic sensor and force the relay OFF when visitors are close.

  Hardware wiring (UNO R4 WiFi):
    Relay module IN pin  -> RELAY_PIN (defaults to D5)
    Relay module VCC/GND -> 5V / GND (respect module voltage limits)
    HC-SR04 TRIG         -> TRIG_PIN (defaults to D6)
    HC-SR04 ECHO         -> ECHO_PIN (defaults to D7) through a voltage divider if required
    HC-SR04 VCC/GND      -> 5V / GND

  Configure WiFi credentials before uploading.
*/

#include <WiFiS3.h>
#include <Arduino_LED_Matrix.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "No secrets.h found; using placeholder WiFi credentials"
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";
#endif

ArduinoLEDMatrix skullMatrix;

static const uint8_t SKULL_BITMAP_READY[8][12] = {
  {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
  {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
  {1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1},
  {1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1},
  {1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1},
  {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
  {0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0},
  {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0}
};

void displayReadySkull() {
  uint8_t rotated[8][12] = {};
  const int srcRows = 8;
  const int srcCols = 12;
  const int destRows = 8;
  const int destCols = 12;

  for (int destRow = 0; destRow < destRows; ++destRow) {
    const int yPrime = (destRow * (srcCols - 1) + ((destRows - 1) / 2)) / (destRows - 1);
    for (int destCol = 0; destCol < destCols; ++destCol) {
      const int xPrime = (destCol * (srcRows - 1) + ((destCols - 1) / 2)) / (destCols - 1);
      const int srcRow = xPrime;
      const int srcCol = (srcCols - 1) - yPrime;
      rotated[destRow][destCol] = SKULL_BITMAP_READY[srcRow][srcCol];
    }
  }

  skullMatrix.renderBitmap(rotated, destRows, destCols);
}

void flashSkull(uint8_t count, unsigned long intervalMs) {
  for (uint8_t i = 0; i < count; ++i) {
    skullMatrix.clear();
    delay(intervalMs);
    displayReadySkull();
    delay(intervalMs);
  }
}

// ==== WiFi configuration =====================================================
const uint16_t WEB_PORT = 80;

// ==== Pin configuration ======================================================
const uint8_t RELAY_PIN = 9;
const bool RELAY_ACTIVE_LEVEL = HIGH;   // Relay FeatherWing is active-high
const uint8_t TRIG_PIN = 6;
const uint8_t ECHO_PIN = 7;

// ==== Timing and behaviour ===================================================
const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 200;
const unsigned long VISITOR_COOLDOWN_MS = 7000;     // Wait after a visitor leaves before reactivating
const float VISITOR_DISTANCE_CM = 100.0f;           // Shut off relay when closer than this
const float SENSOR_MIN_DISTANCE_CM = 5.0f;
const float SENSOR_MAX_DISTANCE_CM = 152.4f;
const unsigned long DISTANCE_LOG_INTERVAL_MS = 1000;
const unsigned long RELAY_PULSE_MS = 150;
const unsigned long SKULL_SEQUENCE_MS = 15000;

// ==== Globals =================================================================
WiFiServer server(WEB_PORT);

bool visitorPresent = false;
unsigned long visitorClearAt = 0;
float lastDistanceCm = 400.0f;

unsigned long lastSensorSampleAt = 0;
unsigned long lastDistanceLogAt = 0;
bool sequenceActive = false;
unsigned long sequenceEndAt = 0;

// ==== Utility prototypes ======================================================
void ensureWifiConnected();
void handleClient(WiFiClient &client);
void sendHttpResponse(WiFiClient &client, int statusCode, const char *statusText, const char *body);
void pulseRelay(const char *reason = nullptr);
void startSequence(const char *source);
void stopSequence(const char *source);
float readDistanceCm();
IPAddress waitForLocalIp(unsigned long timeoutMs = 3000);
void flashSkull(uint8_t count = 3, unsigned long intervalMs = 200);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for USB Serial
  }

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL ? LOW : HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ensureWifiConnected();
  server.begin();

  skullMatrix.begin();
  displayReadySkull();

  IPAddress ip = waitForLocalIp();
  Serial.print("HTTP server started on http://");
  Serial.print(ip);
  Serial.print(":");
  Serial.println(WEB_PORT);
}

void loop() {
  ensureWifiConnected();

  WiFiClient client = server.available();
  if (client) {
    handleClient(client);
    client.stop();
  }

  const unsigned long now = millis();

  if (sequenceActive && now >= sequenceEndAt) {
    sequenceActive = false;
    sequenceEndAt = 0;
    Serial.println("Skull sequence completed (auto timeout)");
  }

  if (now - lastSensorSampleAt >= SENSOR_SAMPLE_INTERVAL_MS) {
    lastSensorSampleAt = now;
    const float distance = readDistanceCm();
    if (distance > 0) {
      lastDistanceCm = distance;
      if (now - lastDistanceLogAt >= DISTANCE_LOG_INTERVAL_MS) {
        lastDistanceLogAt = now;
        Serial.print("Sensor distance: ");
        Serial.print(distance, 1);
        Serial.println(" cm");
      }

      if (distance <= VISITOR_DISTANCE_CM) {
        if (!visitorPresent) {
          Serial.println("Visitor detected nearby – preparing to stop sequence");
        }
        visitorPresent = true;
        visitorClearAt = now + VISITOR_COOLDOWN_MS;
        if (sequenceActive) {
          stopSequence("Visitor proximity");
        }
      } else if (visitorPresent && now >= visitorClearAt) {
        visitorPresent = false;
      }
    }
  }
}

// ==== WiFi helpers ============================================================
void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  int attempt = 0;
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) {
    attempt++;
    Serial.print("  Connection attempt ");
    Serial.print(attempt);
    Serial.println(" failed. Retrying in 2 seconds...");
    delay(2000);
  }

  IPAddress ip = waitForLocalIp();
  Serial.print("Controller IP: ");
  Serial.println(ip);
}

// ==== HTTP handling ===========================================================
void handleClient(WiFiClient &client) {
  char requestLine[128] = {0};
  size_t index = 0;

  // Read the first request line
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        break;
      }
      if (index < sizeof(requestLine) - 1) {
        requestLine[index++] = c;
      }
    }
  }

  String req(requestLine);
  Serial.print("HTTP request: ");
  Serial.println(req);
  flashSkull();

  bool handled = false;
  const unsigned long now = millis();

  if (req.startsWith("POST /activate")) {
    if (visitorPresent) {
      sendHttpResponse(client, 423, "Locked", "{\"status\":\"visitor-nearby\"}");
    } else if (sequenceActive) {
      sendHttpResponse(client, 409, "Conflict", "{\"status\":\"already-running\"}");
    } else {
      startSequence("HTTP /activate");
      sendHttpResponse(client, 200, "OK", "{\"status\":\"activated\"}");
    }
    handled = true;
  } else if (req.startsWith("POST /deactivate")) {
    if (sequenceActive) {
      stopSequence("HTTP /deactivate");
      sendHttpResponse(client, 200, "OK", "{\"status\":\"stopped\"}");
    } else {
      sendHttpResponse(client, 200, "OK", "{\"status\":\"idle\"}");
    }
    handled = true;
  } else if (req.startsWith("GET /status")) {
    char body[160];
    long remaining = 0;
    if (sequenceActive && sequenceEndAt > now) {
      remaining = (long)(sequenceEndAt - now);
    }
    snprintf(body, sizeof(body),
             "{\"sequence\":%s,\"visitor\":%s,\"distance_cm\":%.1f,\"remaining_ms\":%ld}",
             sequenceActive ? "true" : "false",
             visitorPresent ? "true" : "false",
             lastDistanceCm,
             remaining);

    sendHttpResponse(client, 200, "OK", body);
    handled = true;
  }

  if (!handled) {
    sendHttpResponse(client, 404, "Not Found", "{\"error\":\"unknown-endpoint\"}");
  }
}

void sendHttpResponse(WiFiClient &client, int statusCode, const char *statusText, const char *body) {
  client.print("HTTP/1.1 ");
  client.print(statusCode);
  client.print(" ");
  client.println(statusText);
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(strlen(body));
  client.println("Connection: close");
  client.println();
  client.print(body);
}

// ==== Relay helpers ===========================================================
void pulseRelay(const char *reason) {
  if (reason) {
    Serial.print("Relay pulse requested: ");
    Serial.println(reason);
  }
  Serial.print("Relay pulse (pin ");
  Serial.print(RELAY_PIN);
  Serial.println(") active");
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL);
  delay(RELAY_PULSE_MS);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL ? LOW : HIGH);
  Serial.println("Relay pulse complete");
}

void startSequence(const char *source) {
  if (sequenceActive) {
    return;
  }
  pulseRelay(source ? source : "start");
  sequenceActive = true;
  sequenceEndAt = millis() + SKULL_SEQUENCE_MS;
  Serial.println("Skull sequence marked active");
}

void stopSequence(const char *source) {
  if (!sequenceActive) {
    return;
  }
  pulseRelay(source ? source : "stop");
  sequenceActive = false;
  sequenceEndAt = 0;
  Serial.println("Skull sequence marked inactive");
}

// ==== Sensor helpers ==========================================================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25 ms timeout (~4 m)
  if (duration == 0) {
    return -1.0f;
  }

  float distance = (duration / 2.0f) * 0.0343f; // Speed of sound 343 m/s
  if (distance < SENSOR_MIN_DISTANCE_CM || distance > SENSOR_MAX_DISTANCE_CM) {
    return -1.0f;
  }
  return distance;
}

IPAddress waitForLocalIp(unsigned long timeoutMs) {
  IPAddress ip = WiFi.localIP();
  unsigned long start = millis();
  while (ip == IPAddress(0, 0, 0, 0) && millis() - start < timeoutMs) {
    delay(50);
    ip = WiFi.localIP();
  }
  return ip;
}
