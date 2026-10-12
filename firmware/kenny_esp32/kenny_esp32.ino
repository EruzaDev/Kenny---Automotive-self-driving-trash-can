/* KENNY commissioning firmware: ESP32-WROOM, 4 encoder motors, 2 L298N boards.
 * Arduino-ESP32 3.x. See docs/ESP32_FIRMWARE.md BEFORE connecting power.
 * Motors stay disabled until ARM is sent. Expected x4 CPR: 11 PPR * 30:1 * 4 = 1320.
 */
#include <Arduino.h>
#include "protocol.h"

enum Wheel { LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR, WHEEL_COUNT };
// Every motor channel has independent direction and PWM outputs.
constexpr uint8_t ESTOP_PIN = 15;  // Normally-closed auxiliary contact to GND.
constexpr uint32_t DEFAULT_ENCODER_CPR = 1320;  // Supplier 333 RPM model: 11 PPR * 30:1 * x4.
constexpr float WHEEL_RADIUS_M = 0.040f;
constexpr float TRACK_M = 0.235f;  // Provisional; measure on the four-wheel chassis.
constexpr float MAX_WHEEL_MPS = 0.15f;
constexpr uint32_t CONTROL_US = 10000;  // 100 Hz wheel-speed control.
constexpr uint32_t TELEMETRY_MS = 100;
constexpr uint32_t NO_ENCODER_MS = 500;
constexpr int MAX_PWM = 90;  // Conservative commissioning cap (0..255).
constexpr float KP = 1.5f, KI = 0.5f;  // Starting values, NOT tuned gains.
constexpr float NOMINAL_RPM = 333.0f;  // Listing value, NOT calibrated loaded RPM.

enum Fault : uint32_t {
  WATCHDOG = 1, ESTOP = 2, BAD_COMMAND = 4, ENCODER = 8, UNCONFIGURED = 16
};
struct Motor {
  uint8_t pwmPin, in1, in2, encoderA, encoderB;
  int encoderSign;
  volatile uint32_t count;
  volatile uint8_t previousAB;
  uint32_t previousCount;
  uint32_t cpr;
  float rpm, targetRpm, integral;
  int duty;
  uint32_t missingSince;
  bool monitoringMissing;
  int direction;
};
// PWM, direction IN1/IN2, encoder A/B, encoder sign (+1 or -1).
Motor motors[WHEEL_COUNT] = {
  {25, 16, 17, 34, 27, +1}, {26, 18, 19, 35, 14, +1},
  {32, 21, 22, 36, 13, +1}, {33, 4, 5, 39, 23, +1}
};
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
// DRAM storage keeps the ISR lookup table accessible with the flash cache off.
DRAM_ATTR int8_t quadrature[16] = {0, -1, +1, 0, +1, 0, 0, -1,
                                 -1, 0, 0, +1, 0, +1, -1, 0};
bool armed = false, hasSequence = false;
uint32_t faults = 0, sequence = 0, commandMs = 0;
uint32_t controlUs = 0, telemetryMs = 0;
char line[kenny::LINE_CAPACITY];
unsigned lineLength = 0;
bool discardLine = false;

void IRAM_ATTR encoderInterrupt(void* argument) {
  Motor* motor = static_cast<Motor*>(argument);
  uint8_t ab = (digitalRead(motor->encoderA) << 1) | digitalRead(motor->encoderB);
  portENTER_CRITICAL_ISR(&encoderMux);
  int delta = quadrature[(motor->previousAB << 2) | ab] * motor->encoderSign;
  // Unsigned counters wrap predictably; telemetry prints their signed interpretation.
  motor->count += delta;
  motor->previousAB = ab;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void readCounts(uint32_t result[WHEEL_COUNT]) {
  portENTER_CRITICAL(&encoderMux);
  for (unsigned i = 0; i < WHEEL_COUNT; ++i) result[i] = motors[i].count;
  portEXIT_CRITICAL(&encoderMux);
}

void setDirection(uint8_t in1, uint8_t in2, int direction) {
  digitalWrite(in1, direction > 0 ? HIGH : LOW);
  digitalWrite(in2, direction < 0 ? HIGH : LOW);
}

void zeroDrive() {
  for (auto& motor : motors) {
    ledcWrite(motor.pwmPin, 0);
    motor.targetRpm = 0;
    motor.integral = 0;
    motor.duty = 0;
    motor.monitoringMissing = false;
    setDirection(motor.in1, motor.in2, 0);
    motor.direction = 0;
  }
}

void stopDrive(uint32_t reason) {
  armed = false;
  faults |= reason;
  zeroDrive();
}

bool configured() {
  for (const auto& motor : motors) if (!motor.cpr) return false;
  return true;
}

void telemetry() {
  uint32_t counts[WHEEL_COUNT];
  readCounts(counts);
  // S milliseconds last_sequence armed fault_bits LF RF LR RR RPM_LF RPM_RF RPM_LR RPM_RR
  Serial.printf("S %lu %lu %u %lu %ld %ld %ld %ld %.2f %.2f %.2f %.2f\n",
                static_cast<unsigned long>(millis()), static_cast<unsigned long>(sequence),
                armed ? 1 : 0, static_cast<unsigned long>(faults),
                static_cast<long>(static_cast<int32_t>(counts[0])),
                static_cast<long>(static_cast<int32_t>(counts[1])),
                static_cast<long>(static_cast<int32_t>(counts[2])),
                static_cast<long>(static_cast<int32_t>(counts[3])),
                motors[0].rpm, motors[1].rpm, motors[2].rpm, motors[3].rpm);
}

void velocity(float linear, float angular) {
  float left = linear - angular * TRACK_M / 2;
  float right = linear + angular * TRACK_M / 2;
  float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > MAX_WHEEL_MPS) {
    left *= MAX_WHEEL_MPS / peak;
    right *= MAX_WHEEL_MPS / peak;
  }
  float conversion = 60 / (2 * PI * WHEEL_RADIUS_M);
  motors[LEFT_FRONT].targetRpm = motors[LEFT_REAR].targetRpm = left * conversion;
  motors[RIGHT_FRONT].targetRpm = motors[RIGHT_REAR].targetRpm = right * conversion;
}

void handleLine() {
  kenny::Command command = kenny::parse(line);
  switch (command.kind) {
    case kenny::Kind::Hello:
      Serial.println("KENNY_ESP32_L298N_V1 115200 4WD CPR_X4 WATCHDOG_200MS");
      break;
    case kenny::Kind::Status:
      telemetry();
      break;
    case kenny::Kind::Stop:
      stopDrive(0);
      Serial.println("OK STOP");
      break;
    case kenny::Kind::Configure:
      stopDrive(0);
      for (unsigned i = 0; i < WHEEL_COUNT; ++i) motors[i].cpr = command.counts[i];
      faults = digitalRead(ESTOP_PIN) == HIGH ? static_cast<uint32_t>(ESTOP) : 0U;
      Serial.println("OK CONFIG");
      break;
    case kenny::Kind::Arm:
      zeroDrive();
      if (!configured() || digitalRead(ESTOP_PIN) == HIGH) {
        stopDrive((!configured() ? static_cast<uint32_t>(UNCONFIGURED) : 0U) |
                  (digitalRead(ESTOP_PIN) == HIGH ? static_cast<uint32_t>(ESTOP) : 0U));
        Serial.println("ERR ARM CONFIG_OR_ESTOP");
      } else {
        faults = 0;
        armed = true;
        hasSequence = false;
        commandMs = millis();
        Serial.println("OK ARM");
      }
      break;
    case kenny::Kind::Velocity:
      if (!armed) { Serial.println("ERR DISARMED"); break; }
      if (hasSequence && !kenny::newer(command.sequence, sequence)) {
        // Replayed/out-of-order commands do not refresh the watchdog.
        Serial.println("ERR SEQUENCE");
        break;
      }
      sequence = command.sequence;
      hasSequence = true;
      commandMs = millis();
      velocity(command.linear, command.angular);
      break;
    default:
      stopDrive(BAD_COMMAND);
      Serial.println("ERR COMMAND");
      break;
  }
}

void serialInput() {
  // Bounded work per loop keeps the watchdog/PID responsive during input floods.
  unsigned budget = 128;
  while (budget-- && Serial.available()) {
    char character = Serial.read();
    if (character == '\r') continue;
    if (character == '\n') {
      if (!discardLine && lineLength) {
        line[lineLength] = '\0';
        handleLine();
      }
      lineLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (character < 32 || character > 126 || lineLength >= sizeof(line) - 1) {
        discardLine = true;
        lineLength = 0;
        stopDrive(BAD_COMMAND);
        Serial.println("ERR LINE");
      } else {
        line[lineLength++] = character;
      }
    }
  }
}

void updateControl(float dt, uint32_t nowMs) {
  uint32_t counts[WHEEL_COUNT];
  readCounts(counts);
  for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
    Motor& motor = motors[i];
    int requestedDirection = motor.targetRpm > 0 ? 1 : (motor.targetRpm < 0 ? -1 : 0);
    bool directionChanged = requestedDirection != motor.direction;
    if (directionChanged) {
      // Keep PWM zero for this interval before powering the new direction.
      ledcWrite(motor.pwmPin, 0);
      setDirection(motor.in1, motor.in2, requestedDirection);
      motor.direction = requestedDirection;
      motor.integral = 0;
    }
    int32_t delta = static_cast<int32_t>(counts[i] - motor.previousCount);
    motor.previousCount = counts[i];
    if (motor.cpr) {
      float measured = delta * 60.0f / (motor.cpr * dt);
      motor.rpm = 0.7f * motor.rpm + 0.3f * measured;
    }
    if (!armed || directionChanged || fabsf(motor.targetRpm) < 0.01f) {
      ledcWrite(motor.pwmPin, 0);
      motor.duty = 0;
      motor.integral = 0;
      motor.monitoringMissing = false;
      continue;
    }
    float desired = fabsf(motor.targetRpm);
    float measuredAlongTarget = motor.rpm * (motor.targetRpm > 0 ? 1 : -1);
    float error = desired - measuredAlongTarget;
    float feedforward = desired * 255 / NOMINAL_RPM;
    float candidateIntegral = constrain(motor.integral + error * dt, -100.0f, 100.0f);
    float candidate = feedforward + KP * error + KI * candidateIntegral;
    if ((candidate >= 0 && candidate <= MAX_PWM) ||
        (candidate > MAX_PWM && error < 0) || (candidate < 0 && error > 0)) {
      motor.integral = candidateIntegral;
    }
    motor.duty = static_cast<int>(constrain(feedforward + KP * error + KI * motor.integral,
                                           0.0f, static_cast<float>(MAX_PWM)));
    ledcWrite(motor.pwmPin, motor.duty);
    // Detect missing pulses or rotation opposite to the requested direction.
    if (motor.duty >= 50 && desired >= 5 && delta * (motor.targetRpm > 0 ? 1 : -1) <= 0) {
      if (!motor.monitoringMissing) { motor.missingSince = nowMs; motor.monitoringMissing = true; }
      if (static_cast<uint32_t>(nowMs - motor.missingSince) >= NO_ENCODER_MS) {
        stopDrive(ENCODER);
        return;
      }
    } else {
      motor.monitoringMissing = false;
    }
  }
}

void setup() {
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  bool pwmReady = true;
  for (auto& motor : motors) {
    pinMode(motor.in1, OUTPUT); pinMode(motor.in2, OUTPUT);
    setDirection(motor.in1, motor.in2, 0);
    motor.cpr = DEFAULT_ENCODER_CPR;
    pinMode(motor.pwmPin, OUTPUT);
    digitalWrite(motor.pwmPin, LOW);
    pwmReady = ledcAttach(motor.pwmPin, 20000, 8) && pwmReady;
    ledcWrite(motor.pwmPin, 0);
    // All encoder inputs require suitable external pull-ups/level shifting.
    pinMode(motor.encoderA, INPUT); pinMode(motor.encoderB, INPUT);
    motor.previousAB = (digitalRead(motor.encoderA) << 1) | digitalRead(motor.encoderB);
    attachInterruptArg(motor.encoderA, encoderInterrupt, &motor, CHANGE);
    attachInterruptArg(motor.encoderB, encoderInterrupt, &motor, CHANGE);
  }
  zeroDrive();
  if (digitalRead(ESTOP_PIN) == HIGH) faults |= ESTOP;
  if (!pwmReady) {
    Serial.println("ERR PWM_INIT");
    while (true) { zeroDrive(); delay(100); }
  }
  controlUs = micros();
  Serial.println("KENNY_ESP32_L298N_V1 READY_DISABLED");
}

void loop() {
  uint32_t nowMs = millis();
  if (digitalRead(ESTOP_PIN) == HIGH) stopDrive(ESTOP);
  if (armed && kenny::timedOut(nowMs, commandMs)) stopDrive(WATCHDOG);
  serialInput();
  // Recheck after parsing so a long/stale serial exchange cannot bypass stopping.
  nowMs = millis();
  if (digitalRead(ESTOP_PIN) == HIGH) stopDrive(ESTOP);
  if (armed && kenny::timedOut(nowMs, commandMs)) stopDrive(WATCHDOG);
  uint32_t nowUs = micros();
  uint32_t elapsed = nowUs - controlUs;
  if (elapsed >= CONTROL_US) {
    controlUs = nowUs;
    updateControl(elapsed / 1000000.0f, nowMs);
  }
  if (static_cast<uint32_t>(nowMs - telemetryMs) >= TELEMETRY_MS && Serial.availableForWrite() >= 120) {
    telemetryMs = nowMs;
    telemetry();
  }
  delay(1);
}
