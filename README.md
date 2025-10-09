## Halloween Skull Activation System

This project coordinates two Arduino UNO R4 WiFi boards to animate three Halloween skulls with motion triggers and remote activation.

### Hardware Roles
- **Controller Board** – Hosts an HTTP server, drives a power relay for the skulls, and reads an HC‑SR04 ultrasonic range sensor to automatically shut the relay off when visitors approach.
- **Remote Sensor Board** – Monitors a second HC‑SR04 and calls the controller's webhook whenever motion is detected to trigger an activation.

### Firmware Layout
- `controller_skull_server/controller_skull_server.ino` – WiFi configuration, REST endpoint, relay control logic, and safety shutdown based on the local range sensor.
- `trigger_sensor_client/trigger_sensor_client.ino` – WiFi client that measures distance and posts to the controller when someone passes by.
- `controller_skull_server/secrets.example.h`, `trigger_sensor_client/secrets.example.h` – template headers you can copy to `secrets.h` for local WiFi credentials (kept out of git).

### Wiring Cheatsheet

| Component | Controller Pin | Remote Sensor Pin | Notes |
|-----------|----------------|-------------------|-------|
| Relay Signal | `D5` (configurable) | – | Set `RELAY_ACTIVE_LEVEL` to match your relay logic |
| HC-SR04 TRIG | `D6` | `D6` | Keep cable runs short to reduce noise |
| HC-SR04 ECHO | `D7` | `D7` | Use a resistor divider if your sensor runs at 5 V |
| 5V / GND | 5V / GND | 5V / GND | Share grounds between modules |

### Firmware Setup
1. Copy each `secrets.example.h` to `secrets.h` inside the matching folder. Add your WiFi SSID/password, and set `CONTROLLER_HOST` in the trigger client’s secrets to the controller board IP/hostname. Keep these files local (they're listed in `.gitignore`).
2. Open `controller_skull_server/controller_skull_server.ino` in the Arduino IDE, adjust pins/thresholds if your wiring differs, and upload to the UNO R4 WiFi that will host the relay.
3. Open `trigger_sensor_client/trigger_sensor_client.ino` and upload to the second UNO R4 WiFi.

### Operation
- The controller exposes:
  - `POST /activate` – turns on the relay for `duration` milliseconds (default 8000).
  - `POST /deactivate` – immediate shutoff.
  - `GET /status` – JSON snapshot (`relay`, `visitor`, distance, remaining time).
- When the controller’s HC-SR04 sees a visitor within `VISITOR_DISTANCE_CM` (default 120 cm), it forces the relay off and waits `VISITOR_COOLDOWN_MS` before allowing a new activation.
- The remote sensor samples every ~120 ms, requires several consecutive hits before triggering, and enforces a `RETRIGGER_DELAY_MS` cooldown to avoid spamming the server.

### Testing Tips
- Use the Serial Monitor at 115200 baud on each board to watch WiFi connections, distance readings, and trigger logs.
- Before deploying the sensors, test with shorter thresholds (`DETECTION_DISTANCE_CM`, `VISITOR_DISTANCE_CM`) to confirm logic.
- You can manually activate the controller with `curl -X POST http://<controller-ip>/activate`.
