/* KENNY phone joystick: classic ESP32-WROOM, four encoders, TWO external L298N.
 * Select DIRECT GPIO or the MCP23017 CARRIER PCB in config.h before upload.
 * Arduino-ESP32 3.3.12. Open docs/ESP32_MOBILE_CONTROLLER.md before powering motors.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <esp_system.h>
#include "control.h"
#include "config.h"
#include "web_ui.h"

struct Motor {
  uint8_t pwm, in1, in2, encA, encB;
  int encoderSign;
  volatile uint32_t count;
  volatile uint8_t previousAB;
  uint32_t previousCount;
  float rpm, rampRpm, integral;
  int duty, direction;
  bool monitoring;
  uint32_t missingSince, directionSince;
};
// Order LF, RF, LR, RR. All four direction pairs and enables are independent.
Motor motors[MOTOR_COUNT] = {
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
  // in1/in2 are MCP port-B bit numbers, NOT ESP32 GPIO numbers in this mode.
  {25, 0, 1, 34, 27, +1}, {26, 2, 3, 35, 14, +1},
  {32, 4, 5, 36, 13, +1}, {33, 6, 7, 39, 23, +1}
#else
  {25, 16, 17, 34, 27, +1}, {26, 18, 19, 35, 14, +1},
  {32, 21, 22, 36, 13, +1}, {33, 4, 5, 39, 23, +1}
#endif
};
DRAM_ATTR int8_t quadrature[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
mobile::State control;
struct Snapshot {
  bool armed, estop, ready;
  uint32_t faults, lease, uptime;
  float linear, angular;
  int32_t counts[MOTOR_COUNT];
  float rpm[MOTOR_COUNT];
  int pwm[MOTOR_COUNT];
};
Snapshot sharedSnapshot = {};
struct Ack { uint32_t id; bool accepted; };
QueueHandle_t commands = nullptr, acknowledgements = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
WebServer server(80);
bool hardwareReady = false;
uint32_t requestId = 0, bootId = 0;
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
bool expanderHealthy = false;
uint8_t directionBank = 0;
uint32_t lastExpanderCheck = 0;

bool mcpWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP_ADDRESS); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool mcpRead(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(MCP_ADDRESS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MCP_ADDRESS, uint8_t(1)) != 1) return false;
  value = Wire.read(); return true;
}
bool initializeExpander() {
  // Enables are already low. Normalize BANK=0, then latch zero BEFORE output mode.
  expanderHealthy = mcpWrite(0x05, 0) && mcpWrite(0x0A, 0) && mcpWrite(0x0B, 0) &&
    mcpWrite(0x15, 0) && mcpWrite(0x14, 0) && mcpWrite(0x00, 0xFF) &&
    mcpWrite(0x0C, 0xFF) && mcpWrite(0x01, 0);
  uint8_t directionMode = 0xFF, latch = 0xFF;
  expanderHealthy = expanderHealthy && mcpRead(0x01, directionMode) &&
    mcpRead(0x15, latch) && directionMode == 0 && latch == 0;
  directionBank = 0;
  return expanderHealthy;
}
bool expanderCheck(uint32_t now) {
  if (!expanderHealthy) return false;
  if (static_cast<uint32_t>(now - lastExpanderCheck) < 100) return true;
  lastExpanderCheck = now;
  uint8_t mode, latch;
  expanderHealthy = mcpRead(0x01, mode) && mcpRead(0x15, latch) && mode == 0 && latch == directionBank;
  return expanderHealthy;
}
#endif

bool setDirection(Motor& motor, int direction) {
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
  if (!expanderHealthy) return false;
  uint8_t bits = (1U << motor.in1) | (1U << motor.in2);
  uint8_t next = directionBank & ~bits;
  if (direction > 0) next |= 1U << motor.in1;
  if (direction < 0) next |= 1U << motor.in2;
  if (next == directionBank) return true;
  if (!mcpWrite(0x15, next)) { expanderHealthy = false; return false; }
  directionBank = next; return true;
#else
  digitalWrite(motor.in1, direction > 0 ? HIGH : LOW);
  digitalWrite(motor.in2, direction < 0 ? HIGH : LOW);
  return true;
#endif
}

void IRAM_ATTR encoderInterrupt(void* argument) {
  Motor* motor = static_cast<Motor*>(argument);
  uint8_t ab = (digitalRead(motor->encA) << 1) | digitalRead(motor->encB);
  portENTER_CRITICAL_ISR(&encoderMux);
  motor->count += quadrature[(motor->previousAB << 2) | ab] * motor->encoderSign;
  motor->previousAB = ab;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

bool outputZero(Motor& motor) {
  ledcWrite(motor.pwm, 0);
  motor.duty = 0; motor.direction = 0; motor.rampRpm = 0;
  motor.integral = 0; motor.monitoring = false;
  return setDirection(motor, 0);
}

void allZero() {
  // Disable ALL enables first, independently of potentially stalled I2C writes.
  for (auto& motor : motors) ledcWrite(motor.pwm, 0);
  for (auto& motor : motors) {
    motor.duty = motor.direction = 0; motor.rampRpm = motor.integral = 0; motor.monitoring = false;
#if KENNY_WIRING == KENNY_DIRECT_GPIO
    setDirection(motor, 0);
#endif
  }
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
  if (expanderHealthy && directionBank && !mcpWrite(0x15, 0)) expanderHealthy = false;
  directionBank = 0;
  if (!expanderHealthy) { hardwareReady = false; control.stop(mobile::I2C_DRIVER); }
#endif
}

void directionFailure() { control.stop(mobile::I2C_DRIVER); allZero(); }

void requestUrgentStop() {
  if (motorTaskHandle) xTaskNotify(motorTaskHandle, 1, eSetBits);
}

Snapshot snapshot() {
  portENTER_CRITICAL(&snapshotMux);
  Snapshot result = sharedSnapshot;
  portEXIT_CRITICAL(&snapshotMux);
  return result;
}

void updateMotors(float dt, uint32_t now) {
  uint32_t counts[MOTOR_COUNT];
  portENTER_CRITICAL(&encoderMux);
  for (unsigned i = 0; i < MOTOR_COUNT; ++i) counts[i] = motors[i].count;
  portEXIT_CRITICAL(&encoderMux);
  float left, right;
  mobile::wheelSpeeds(control.linear, control.angular, left, right);
  const float conversion = 60 / (2 * PI * WHEEL_RADIUS);
  for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
    Motor& motor = motors[i];
    int32_t delta = static_cast<int32_t>(counts[i] - motor.previousCount);
    motor.previousCount = counts[i];
    float measured = ENCODER_CPR ? delta * 60.0f / (ENCODER_CPR * dt) : 0;
    motor.rpm = 0.7f * motor.rpm + 0.3f * measured;
    float requested = (i % 2 == 0 ? left : right) * conversion;
    if (!control.armed || fabsf(requested) < 0.01f) {
      if (!outputZero(motor)) { directionFailure(); return; }
      continue;
    }
    int direction = requested > 0 ? 1 : -1;
    if (motor.direction != direction) {
      if (!outputZero(motor)) { directionFailure(); return; }
      motor.direction = direction; motor.directionSince = now;
      // Keep enable low for at least 50 ms before powering a new direction.
      if (!setDirection(motor, direction)) { directionFailure(); return; }
      continue;
    }
    if (static_cast<uint32_t>(now - motor.directionSince) < 50) continue;
    if (motor.rpm * direction < -5) {
      ledcWrite(motor.pwm, 0); motor.duty = 0;
      if (static_cast<uint32_t>(now - motor.directionSince) > 500) {
        control.stop(mobile::ENCODER); allZero(); return;
      }
      continue;
    }
    motor.rampRpm += constrain(requested - motor.rampRpm, -RPM_SLEW_PER_SECOND * dt, RPM_SLEW_PER_SECOND * dt);
    float desired = fabsf(motor.rampRpm);
    float error = desired - motor.rpm * direction;
    float feedforward = desired * 255 / NOMINAL_RPM;
    float candidateI = constrain(motor.integral + error * dt, -100.0f, 100.0f);
    float candidate = feedforward + KP * error + KI * candidateI;
    if ((candidate >= 0 && candidate <= MAX_PWM) ||
        (candidate > MAX_PWM && error < 0) || (candidate < 0 && error > 0)) motor.integral = candidateI;
    motor.duty = static_cast<int>(constrain(feedforward + KP * error + KI * motor.integral,
                                           0.0f, static_cast<float>(MAX_PWM)));
    ledcWrite(motor.pwm, motor.duty);
    if (motor.duty >= 50 && desired >= 5 && delta * direction <= 0) {
      if (!motor.monitoring) { motor.monitoring = true; motor.missingSince = now; }
      if (static_cast<uint32_t>(now - motor.missingSince) >= ENCODER_TIMEOUT_MS) {
        control.stop(mobile::ENCODER); allZero(); return;
      }
    } else motor.monitoring = false;
  }
}

void motorTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t previousUs = micros();
  while (true) {
    uint32_t now = millis(), currentUs = micros();
    uint32_t elapsedUs = currentUs - previousUs; previousUs = currentUs;
    bool estop = digitalRead(ESTOP_PIN) == HIGH;
    control.tick(now, estop);
    bool loopLate = elapsedUs > 100000;
    if (loopLate) control.stop(mobile::TIMING);
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
    if (!expanderCheck(now)) directionFailure();
#endif
    mobile::Command command;
    uint32_t notification = 0;
    if (xTaskNotifyWait(0, UINT32_MAX, &notification, 0) == pdTRUE && notification) {
      control.stop(); allZero();
      // No previously queued ARM/motion may survive an urgent stop.
      while (xQueueReceive(commands, &command, 0) == pdTRUE) {
        Ack ack{command.id, command.kind == mobile::Kind::Stop};
        xQueueSend(acknowledgements, &ack, 0);
      }
    }
    for (unsigned budget = 0; budget < 8 && xQueueReceive(commands, &command, 0) == pdTRUE; ++budget) {
      bool accepted = control.apply(command, now, estop, hardwareReady && !loopLate);
      if (!control.armed) allZero();
      Ack ack{command.id, accepted};
      xQueueSend(acknowledgements, &ack, 0);
    }
    if (hardwareReady) updateMotors(fmaxf(elapsedUs / 1000000.0f, 0.001f), now);
    Snapshot next = {};
    next.armed = control.armed; next.estop = estop; next.ready = hardwareReady;
    next.faults = control.faults; next.lease = control.lease; next.uptime = now;
    next.linear = control.linear; next.angular = control.angular;
    portENTER_CRITICAL(&encoderMux);
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) next.counts[i] = static_cast<int32_t>(motors[i].count);
    portEXIT_CRITICAL(&encoderMux);
    for (unsigned i = 0; i < MOTOR_COUNT; ++i) { next.rpm[i] = motors[i].rpm; next.pwm[i] = motors[i].duty; }
    portENTER_CRITICAL(&snapshotMux); sharedSnapshot = next; portEXIT_CRITICAL(&snapshotMux);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// HTTP runs on the Arduino loop task; it never directly drives GPIO/LEDC.
bool dispatch(mobile::Command& command) {
  command.id = ++requestId; command.receivedMs = millis();
  if (xQueueSend(commands, &command, 0) != pdTRUE) { requestUrgentStop(); return false; }
  uint32_t started = millis();
  Ack ack;
  while (static_cast<uint32_t>(millis() - started) < 100) {
    if (xQueueReceive(acknowledgements, &ack, pdMS_TO_TICKS(5)) == pdTRUE && ack.id == command.id) return ack.accepted;
  }
  // Prevent a late ARM/drive acknowledgement from leaving motion enabled.
  requestUrgentStop();
  return false;
}

void jsonResponse(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void stopHandler() {
  requestUrgentStop();
  mobile::Command stop;
  bool accepted = dispatch(stop);
  jsonResponse(accepted ? 200 : 503, accepted ? "{\"ok\":true}" : "{\"ok\":false}");
}

void armHandler() {
  mobile::Command command; command.kind = mobile::Kind::Arm;
  do { command.lease = esp_random(); } while (!command.lease);
  bool accepted = dispatch(command);
  if (accepted) jsonResponse(200, "{\"ok\":true,\"lease\":" + String(command.lease) + "}");
  else jsonResponse(409, "{\"ok\":false,\"error\":\"Already armed, E-stop open, or controller not ready\"}");
}

void driveHandler() {
  mobile::Command command; command.kind = mobile::Kind::Drive;
  if (!server.hasArg("lease") || !server.hasArg("seq") || !server.hasArg("v") || !server.hasArg("w") ||
      !mobile::readUnsigned(server.arg("lease").c_str(), command.lease) ||
      !mobile::readUnsigned(server.arg("seq").c_str(), command.sequence) ||
      !mobile::readFloat(server.arg("v").c_str(), command.linear) ||
      !mobile::readFloat(server.arg("w").c_str(), command.angular)) {
    requestUrgentStop(); mobile::Command stop; dispatch(stop);
    jsonResponse(400, "{\"ok\":false,\"error\":\"Invalid command\"}"); return;
  }
  bool accepted = dispatch(command);
  jsonResponse(accepted ? 200 : 409, accepted ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"Disarmed, expired session or invalid command\"}");
}

void statusHandler() {
  Snapshot s = snapshot();
  char body[650];
  snprintf(body, sizeof(body),
    "{\"armed\":%s,\"estop\":%s,\"ready\":%s,\"faults\":%lu,\"boot\":%lu,\"uptime\":%lu,"
    "\"wiring\":\"%s\",\"v\":%.4f,\"w\":%.4f,\"cpr\":%lu,\"maxV\":%.3f,\"maxW\":%.3f,\"watchdog\":%lu,"
    "\"counts\":[%ld,%ld,%ld,%ld],\"rpm\":[%.2f,%.2f,%.2f,%.2f],\"pwm\":[%d,%d,%d,%d]}",
    s.armed?"true":"false", s.estop?"true":"false", s.ready?"true":"false",
    static_cast<unsigned long>(s.faults), static_cast<unsigned long>(bootId), static_cast<unsigned long>(s.uptime),
    WIRING_NAME, s.linear, s.angular, static_cast<unsigned long>(ENCODER_CPR), mobile::MAX_LINEAR, mobile::MAX_ANGULAR,
    static_cast<unsigned long>(mobile::WATCHDOG_MS),
    static_cast<long>(s.counts[0]), static_cast<long>(s.counts[1]), static_cast<long>(s.counts[2]), static_cast<long>(s.counts[3]),
    s.rpm[0],s.rpm[1],s.rpm[2],s.rpm[3],s.pwm[0],s.pwm[1],s.pwm[2],s.pwm[3]);
  jsonResponse(200, body);
}

void setup() {
  Serial.begin(115200); pinMode(ESTOP_PIN, INPUT_PULLUP);
  hardwareReady = ENCODER_CPR > 0;
  for (auto& motor : motors) {
    pinMode(motor.pwm, OUTPUT); digitalWrite(motor.pwm, LOW);
#if KENNY_WIRING == KENNY_DIRECT_GPIO
    pinMode(motor.in1, OUTPUT); pinMode(motor.in2, OUTPUT);
    setDirection(motor, 0);
#endif
    hardwareReady = ledcAttach(motor.pwm, 20000, 8) && hardwareReady;
    ledcWrite(motor.pwm, 0);
    pinMode(motor.encA, INPUT); pinMode(motor.encB, INPUT);
    motor.previousAB = (digitalRead(motor.encA) << 1) | digitalRead(motor.encB);
    attachInterruptArg(motor.encA, encoderInterrupt, &motor, CHANGE);
    attachInterruptArg(motor.encB, encoderInterrupt, &motor, CHANGE);
  }
  if (!hardwareReady) control.stop(mobile::PWM_INIT | mobile::CONFIGURATION);
#if KENNY_WIRING == KENNY_CARRIER_MCP23017
  pinMode(HX_DOUT, INPUT); pinMode(HX_SCK, OUTPUT); digitalWrite(HX_SCK, LOW);
  Wire.begin(I2C_SDA, I2C_SCL, 100000); Wire.setTimeOut(10);
  if (!initializeExpander()) { hardwareReady = false; control.stop(mobile::I2C_DRIVER); }
#endif
  commands = xQueueCreate(8, sizeof(mobile::Command));
  acknowledgements = xQueueCreate(8, sizeof(Ack));
  if (!commands || !acknowledgements ||
      xTaskCreatePinnedToCore(motorTask, "kenny_motors", 8192, nullptr, 3, &motorTaskHandle, 1) != pdPASS) {
    allZero(); Serial.println("Motor task init failed; disabled."); while(true) delay(100);
  }
  bootId = esp_random();
  WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 6, false, 2)) {
    Serial.println("Wi-Fi AP failed; motors remain disabled."); while(true) delay(100);
  }
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("X-Content-Type-Options", "nosniff");
    server.send_P(200, "text/html", KENNY_WEB_UI);
  });
  server.on("/api/status", HTTP_GET, statusHandler);
  server.on("/api/arm", HTTP_POST, armHandler);
  server.on("/api/drive", HTTP_POST, driveHandler);
  server.on("/api/stop", HTTP_POST, stopHandler);
  server.onNotFound([]() { server.send(404, "text/plain", "Open http://192.168.4.1/"); });
  server.begin();
  Serial.printf("KENNY mobile ready. Wi-Fi: %s | URL: http://%s/ | %s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str(), WIRING_NAME);
}

void loop() {
  server.handleClient();
  // USB serial is monitoring only in this sketch; no competing motion source.
  delay(1);
}
