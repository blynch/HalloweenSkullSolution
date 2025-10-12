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
const bool RELAY_ACTIVE_LEVEL = HIGH;   // Set to LOW if your relay is active-low
const uint8_t TRIG_PIN = 6;
const uint8_t ECHO_PIN = 7;

// ==== Timing and behaviour ===================================================
const unsigned long DEFAULT_ACTIVATION_MS = 8000;   // Relay on time when /activate is called
const unsigned long MIN_ACTIVATION_MS = 1000;
const unsigned long MAX_ACTIVATION_MS = 30000;
const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 200;
const unsigned long VISITOR_COOLDOWN_MS = 7000;     // Wait after a visitor leaves before reactivating
const float VISITOR_DISTANCE_CM = 100.0f;           // Shut off relay when closer than this
const float SENSOR_MIN_DISTANCE_CM = 5.0f;
const float SENSOR_MAX_DISTANCE_CM = 152.4f;
const unsigned long DISTANCE_LOG_INTERVAL_MS = 1000;

// ==== Globals =================================================================
WiFiServer server(WEB_PORT);

bool relayOn = false;
unsigned long relayOffAt = 0;

bool visitorPresent = false;
unsigned long visitorClearAt = 0;
float lastDistanceCm = 400.0f;

unsigned long lastSensorSampleAt = 0;
unsigned long lastDistanceLogAt = 0;

// ==== Utility prototypes ======================================================
void ensureWifiConnected();
void handleClient(WiFiClient &client);
void sendHttpResponse(WiFiClient &client, int statusCode, const char *statusText, const char *body);
void turnRelayOn(unsigned long durationMs);
void turnRelayOff();
float readDistanceCm();
unsigned long clampActivationDuration(long requestMs);
IPAddress waitForLocalIp(unsigned long timeoutMs = 3000);
void flashSkull(uint8_t count = 3, unsigned long intervalMs = 200);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for USB Serial
  }

  pinMode(RELAY_PIN, OUTPUT);
  turnRelayOff();

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

  if (relayOn && now >= relayOffAt) {
    Serial.println("Relay timeout reached, turning off");
    turnRelayOff();
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
        visitorPresent = true;
        visitorClearAt = now + VISITOR_COOLDOWN_MS;
        if (relayOn) {
          Serial.println("Visitor detected nearby – relay forced off");
          turnRelayOff();
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

  if (req.startsWith("POST /activate")) {
    long duration = DEFAULT_ACTIVATION_MS;

    int qsIndex = req.indexOf("duration=");
    if (qsIndex >= 0) {
      duration = req.substring(qsIndex + 9).toInt();
    }

    duration = clampActivationDuration(duration);

    if (visitorPresent) {
      sendHttpResponse(client, 423, "Locked", "{\"status\":\"visitor-nearby\"}");
    } else if (relayOn) {
      relayOffAt = millis() + duration;
      sendHttpResponse(client, 202, "Accepted", "{\"status\":\"extended\"}");
    } else {
      turnRelayOn(duration);
      sendHttpResponse(client, 200, "OK", "{\"status\":\"activated\"}");
    }
    handled = true;
  } else if (req.startsWith("POST /deactivate")) {
    turnRelayOff();
    sendHttpResponse(client, 200, "OK", "{\"status\":\"deactivated\"}");
    handled = true;
  } else if (req.startsWith("GET /status")) {
    char body[160];
    snprintf(body, sizeof(body),
             "{\"relay\":%s,\"visitor\":%s,\"distance_cm\":%.1f,\"timeout_ms\":%ld}",
             relayOn ? "true" : "false",
             visitorPresent ? "true" : "false",
             lastDistanceCm,
             relayOn ? (long)(relayOffAt - millis()) : 0L);

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
void turnRelayOn(unsigned long durationMs) {
  Serial.print("Relay ON signal (pin ");
  Serial.print(RELAY_PIN);
  Serial.println(") asserted");
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL);
  relayOn = true;
  relayOffAt = millis() + durationMs;
  Serial.print("Relay ON for ");
  Serial.print(durationMs);
  Serial.println(" ms");
}

void turnRelayOff() {
  Serial.print("Relay OFF signal (pin ");
  Serial.print(RELAY_PIN);
  Serial.println(") asserted");
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL ? LOW : HIGH);
  relayOn = false;
  relayOffAt = 0;
  Serial.println("Relay OFF");
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
  return distance;
}

unsigned long clampActivationDuration(long requestMs) {
  if (requestMs < (long)MIN_ACTIVATION_MS) {
    return MIN_ACTIVATION_MS;
  }
  if (requestMs > (long)MAX_ACTIVATION_MS) {
    return MAX_ACTIVATION_MS;
  }
  return (unsigned long)requestMs;
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
