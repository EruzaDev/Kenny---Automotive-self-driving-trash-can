/*
 * KENNY — Wheel Odometer, Diagnostics & Closed-Loop 1-Revolution Accuracy Firmware
 *
 * Matches the hardware pinout in docs/ESP32_MOTOR_WIRING.md:
 *   - Two 38-pin ESP32-WROOM-32 boards (LEFT side and RIGHT side)
 *   - Two L298N dual H-bridge drivers
 *   - Four JGB37-520 encoder gearmotors with 80 mm wheels (r = 0.04 m)
 *
 * Control Wiring (same on both Left and Right ESP32):
 *   Motor 1 (OUT1/OUT2): ENA = GPIO32, IN1 = GPIO23, IN2 = GPIO13
 *   Motor 2 (OUT3/OUT4): ENB = GPIO27, IN3 = GPIO19, IN4 = GPIO4
 *
 * Encoder Wiring (docs/ESP32_MOTOR_WIRING.md):
 *   LEFT ESP32 (M1 left rear, M2 left front):
 *     Motor 1: A = GPIO34, B = GPIO25
 *     Motor 2: A = GPIO18, B = GPIO21
 *   RIGHT ESP32:
 *     Motor 1: A = GPIO34, B = GPIO25
 *     Motor 2: A = GPIO18, B = GPIO21
 *   Note: GPIO34 is an input-only pin on ESP32 and has NO internal pull-up.
 *
 * Serial Protocol (115200 baud, \n line endings):
 *   Commands:
 *     ZERO                                     - Reset all encoder counts, edge counters, and glitches
 *     STOP (or S)                              - Immediately stop motors and cancel any active test
 *     SIDE LEFT | SIDE RIGHT                   - Select Left or Right ESP32 encoder pin mapping
 *     CPR <counts>                             - Set quadrature counts per wheel revolution (default 1320)
 *     RADIUS <meters>                          - Set wheel radius in meters (default 0.04)
 *     INVERT <m1_sign> <m2_sign>               - Set encoder sign multipliers (+1 or -1)
 *     DEADBAND <min_pwm>                       - Set minimum closed-loop approach PWM (default 75)
 *     JOG <m1|m2|both> <pwm> [ms]              - Open-loop timed motor jog (-255..255, max 5000 ms)
 *     SPIN_REV <m1|m2|both> <revs> [max_pwm]   - Closed-loop spin by exact <revs> (e.g. 1.000) with
 *                                                deceleration ramp, active H-bridge brake, settle
 *                                                measurement, and REV_RESULT accuracy report
 *     HELP                                     - Print command reference
 */

#include <Arduino.h>
#include <math.h>

#ifndef DEFAULT_SIDE_LEFT
#define DEFAULT_SIDE_LEFT 1  // 1 = Left ESP32, 0 = Right ESP32
#endif

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t TELEMETRY_PERIOD_MS = 50;  // 20 Hz telemetry stream
static const uint32_t MAX_JOG_DURATION_MS = 5000;
static const uint32_t DEFAULT_JOG_DURATION_MS = 500;
static const uint32_t SPIN_REV_TIMEOUT_MS = 8000;
static const uint32_t BRAKE_HOLD_MS = 80;
static const uint32_t SETTLE_WAIT_MS = 350;

// L298N Motor Driver Pins (identical on Left and Right ESP32)
static const int PIN_ENA = 32;
static const int PIN_IN1 = 23;
static const int PIN_IN2 = 13;

static const int PIN_ENB = 27;
static const int PIN_IN3 = 19;
static const int PIN_IN4 = 4;

// Active Encoder Pin Assignments
static bool is_left_side = (DEFAULT_SIDE_LEFT != 0);
static bool use_internal_pullups = true;
static int pin_m1_a = 34;
static int pin_m1_b = 25;
static int pin_m2_a = 18;
static int pin_m2_b = 21;

// Odometry & Control Parameters
static float counts_per_rev = 1320.0f;
static float wheel_radius_m = 0.04f;
static int8_t m1_sign = 1;
static int8_t m2_sign = 1;
static int min_approach_pwm = 75;

// Quadrature Decoder State (updated inside IRAM_ATTR ISRs)
volatile int64_t isr_m1_ticks = 0;
volatile int64_t isr_m2_ticks = 0;
volatile uint32_t isr_m1_a_edges = 0;
volatile uint32_t isr_m1_b_edges = 0;
volatile uint32_t isr_m2_a_edges = 0;
volatile uint32_t isr_m2_b_edges = 0;
volatile uint32_t isr_m1_glitches = 0;
volatile uint32_t isr_m2_glitches = 0;
volatile uint8_t isr_m1_prev_ab = 0;
volatile uint8_t isr_m2_prev_ab = 0;

// 4x Quadrature Lookup Table indexed by ((prev_ab << 2) | curr_ab)
static const int8_t DRAM_ATTR QUAD_TABLE[16] = {
     0, -1, +1,  2,
    +1,  0,  2, -1,
    -1,  2,  0, +1,
     2, +1, -1,  0
};

// Open-loop Jog State
static int current_m1_pwm = 0;
static int current_m2_pwm = 0;
static bool jog_active = false;
static uint32_t jog_deadline_ms = 0;

// Closed-loop 1-Revolution Target Spin State Machine
enum SpinPhase {
  SPIN_IDLE = 0,
  SPIN_MOVING = 1,
  SPIN_BRAKING = 2,
  SPIN_SETTLING = 3
};

struct ClosedLoopAxis {
  SpinPhase phase;
  float target_revs;
  int64_t start_ticks;
  int64_t target_delta_ticks;
  int64_t brake_delta_ticks;
  int max_pwm;
  uint32_t start_ms;
  uint32_t brake_start_ms;
  uint32_t settle_start_ms;
  uint32_t start_glitches;
  float peak_rpm;
  const char* status_code;
};

static ClosedLoopAxis cl_m1 = {SPIN_IDLE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0f, "OK"};
static ClosedLoopAxis cl_m2 = {SPIN_IDLE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0f, "OK"};

// Telemetry State
static uint32_t last_telemetry_ms = 0;
static int64_t prev_m1_ticks = 0;
static int64_t prev_m2_ticks = 0;
static float latest_m1_rpm = 0.0f;
static float latest_m2_rpm = 0.0f;

// Serial Input Buffer
static char cmd_buf[160];
static size_t cmd_len = 0;

// ============================================================================
// Quadrature Interrupt Service Routines
// ============================================================================
void IRAM_ATTR onMotor1EncoderEdge() {
  uint8_t a = (uint8_t)digitalRead(pin_m1_a);
  uint8_t b = (uint8_t)digitalRead(pin_m1_b);
  uint8_t curr_ab = (uint8_t)((a << 1) | b);
  uint8_t prev_ab = isr_m1_prev_ab;
  uint8_t diff = prev_ab ^ curr_ab;
  if (diff & 0x02) isr_m1_a_edges++;
  if (diff & 0x01) isr_m1_b_edges++;

  uint8_t idx = (uint8_t)((prev_ab << 2) | curr_ab);
  isr_m1_prev_ab = curr_ab;

  int8_t step = QUAD_TABLE[idx];
  if (step == 2) {
    isr_m1_glitches++;
  } else if (step != 0) {
    isr_m1_ticks += (int64_t)(step * m1_sign);
  }
}

void IRAM_ATTR onMotor2EncoderEdge() {
  uint8_t a = (uint8_t)digitalRead(pin_m2_a);
  uint8_t b = (uint8_t)digitalRead(pin_m2_b);
  uint8_t curr_ab = (uint8_t)((a << 1) | b);
  uint8_t prev_ab = isr_m2_prev_ab;
  uint8_t diff = prev_ab ^ curr_ab;
  if (diff & 0x02) isr_m2_a_edges++;
  if (diff & 0x01) isr_m2_b_edges++;

  uint8_t idx = (uint8_t)((prev_ab << 2) | curr_ab);
  isr_m2_prev_ab = curr_ab;

  int8_t step = QUAD_TABLE[idx];
  if (step == 2) {
    isr_m2_glitches++;
  } else if (step != 0) {
    isr_m2_ticks += (int64_t)(step * m2_sign);
  }
}

// ============================================================================
// Hardware Setup & Motor Control Helpers
// ============================================================================
static int pinInputMode(int gpio_pin) {
  if (gpio_pin >= 34) {
    return INPUT;  // GPIO34..39 are input-only and lack internal pull-ups
  }
  return use_internal_pullups ? INPUT_PULLUP : INPUT;
}

void setMotor1Coast() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  current_m1_pwm = 0;
}

void setMotor2Coast() {
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  current_m2_pwm = 0;
}

void setMotor1ActiveBrake() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_ENA, 255);
  current_m1_pwm = 0;
}

void setMotor2ActiveBrake() {
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENB, 255);
  current_m2_pwm = 0;
}

void setMotor1Pwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, pwm);
  } else if (pwm < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -pwm);
  } else {
    setMotor1Coast();
  }
  current_m1_pwm = pwm;
}

void setMotor2Pwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  // The left-front motor is mounted with opposite drive polarity. Keep the
  // serial command and telemetry signs logical; reverse only its H-bridge pins.
  int drive_pwm = is_left_side ? -pwm : pwm;
  if (drive_pwm > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, drive_pwm);
  } else if (drive_pwm < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENB, -drive_pwm);
  } else {
    setMotor2Coast();
  }
  current_m2_pwm = pwm;
}

void stopMotors() {
  setMotor1Coast();
  setMotor2Coast();
  jog_active = false;
  cl_m1.phase = SPIN_IDLE;
  cl_m2.phase = SPIN_IDLE;
}

void zeroCounters() {
  noInterrupts();
  isr_m1_ticks = 0;
  isr_m2_ticks = 0;
  isr_m1_a_edges = 0;
  isr_m1_b_edges = 0;
  isr_m2_a_edges = 0;
  isr_m2_b_edges = 0;
  isr_m1_glitches = 0;
  isr_m2_glitches = 0;
  isr_m1_prev_ab = (uint8_t)(((uint8_t)digitalRead(pin_m1_a) << 1) | (uint8_t)digitalRead(pin_m1_b));
  isr_m2_prev_ab = (uint8_t)(((uint8_t)digitalRead(pin_m2_a) << 1) | (uint8_t)digitalRead(pin_m2_b));
  interrupts();
  prev_m1_ticks = 0;
  prev_m2_ticks = 0;
}

void configureEncoderPins(bool left_side) {
  detachInterrupt(digitalPinToInterrupt(pin_m1_a));
  detachInterrupt(digitalPinToInterrupt(pin_m1_b));
  detachInterrupt(digitalPinToInterrupt(pin_m2_a));
  detachInterrupt(digitalPinToInterrupt(pin_m2_b));

  is_left_side = left_side;
  if (is_left_side) {
    pin_m1_a = 34;
    pin_m1_b = 25;
    pin_m2_a = 18;
    pin_m2_b = 21;
  } else {
    pin_m1_a = 34;
    pin_m1_b = 25;
    pin_m2_a = 18;
    pin_m2_b = 21;
  }

  pinMode(pin_m1_a, pinInputMode(pin_m1_a));
  pinMode(pin_m1_b, pinInputMode(pin_m1_b));
  pinMode(pin_m2_a, pinInputMode(pin_m2_a));
  pinMode(pin_m2_b, pinInputMode(pin_m2_b));

  zeroCounters();

  attachInterrupt(digitalPinToInterrupt(pin_m1_a), onMotor1EncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_m1_b), onMotor1EncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_m2_a), onMotor2EncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_m2_b), onMotor2EncoderEdge, CHANGE);
}

// ============================================================================
// Closed-Loop 1-Revolution Target Spin Controller
// ============================================================================
void startClosedLoopSpin(ClosedLoopAxis& ax, int64_t curr_ticks, uint32_t curr_glitches,
                         float revs, int max_pwm, uint32_t now_ms) {
  ax.phase = SPIN_MOVING;
  ax.target_revs = revs;
  ax.start_ticks = curr_ticks;
  ax.target_delta_ticks = (int64_t)llround((double)revs * (double)counts_per_rev);
  ax.brake_delta_ticks = 0;
  ax.max_pwm = constrain(abs(max_pwm), min_approach_pwm, 255);
  ax.start_ms = now_ms;
  ax.brake_start_ms = now_ms;
  ax.settle_start_ms = now_ms;
  ax.start_glitches = curr_glitches;
  ax.peak_rpm = 0.0f;
  ax.status_code = "COMPLETED";
}

void emitRevResult(const char* motor_label, const ClosedLoopAxis& ax,
                   int64_t curr_ticks, uint32_t curr_glitches, uint32_t now_ms) {
  int64_t actual_ticks = curr_ticks - ax.start_ticks;
  int64_t error_ticks = actual_ticks - ax.target_delta_ticks;
  int64_t coast_ticks = actual_ticks - ax.brake_delta_ticks;
  float actual_revs = (float)actual_ticks / counts_per_rev;
  float error_revs = actual_revs - ax.target_revs;
  float error_deg = error_revs * 360.0f;
  float circumference_mm = 2.0f * (float)M_PI * wheel_radius_m * 1000.0f;
  float error_mm = error_revs * circumference_mm;
  float denom = fabsf(ax.target_revs) > 1e-4f ? fabsf(ax.target_revs) : 1.0f;
  float accuracy_pct = max(0.0f, (1.0f - (fabsf(error_revs) / denom)) * 100.0f);
  uint32_t elapsed_ms = ax.settle_start_ms - ax.start_ms;
  uint32_t glitches_delta = curr_glitches - ax.start_glitches;

  Serial.printf(
      "REV_RESULT motor=%s status=%s target_revs=%.4f actual_revs=%.4f "
      "target_ticks=%lld actual_ticks=%lld error_ticks=%+lld error_revs=%+.4f "
      "error_deg=%+.2f error_mm=%+.2f accuracy_pct=%.2f brake_ticks=%lld "
      "coast_ticks=%+lld elapsed_ms=%lu peak_rpm=%.1f glitches=%lu\n",
      motor_label,
      ax.status_code,
      ax.target_revs,
      actual_revs,
      (long long)ax.target_delta_ticks,
      (long long)actual_ticks,
      (long long)error_ticks,
      error_revs,
      error_deg,
      error_mm,
      accuracy_pct,
      (long long)ax.brake_delta_ticks,
      (long long)coast_ticks,
      (unsigned long)elapsed_ms,
      ax.peak_rpm,
      (unsigned long)glitches_delta);
}

void updateClosedLoopAxis(ClosedLoopAxis& ax, const char* motor_label,
                          int64_t curr_ticks, uint32_t curr_glitches,
                          float curr_rpm, uint32_t now_ms,
                          void (*setPwmFn)(int), void (*setBrakeFn)(), void (*setCoastFn)()) {
  if (ax.phase == SPIN_IDLE) {
    return;
  }

  if (fabsf(curr_rpm) > ax.peak_rpm) {
    ax.peak_rpm = fabsf(curr_rpm);
  }

  int64_t progressed_ticks = curr_ticks - ax.start_ticks;
  int dir = (ax.target_delta_ticks >= 0) ? 1 : -1;
  int64_t signed_progress = progressed_ticks * dir;
  int64_t abs_target = llabs(ax.target_delta_ticks);
  int64_t remaining = abs_target - signed_progress;

  if (ax.phase == SPIN_MOVING) {
    // 1. Check if target reached
    if (remaining <= 0 || abs_target == 0) {
      ax.brake_delta_ticks = progressed_ticks;
      ax.brake_start_ms = now_ms;
      ax.settle_start_ms = now_ms;
      ax.phase = SPIN_BRAKING;
      setBrakeFn();
      return;
    }

    // 2. Check for reverse-wired encoder runaway (moving >0.15 rev in wrong direction)
    int64_t runaway_threshold = (int64_t)max(40.0f, 0.15f * counts_per_rev);
    if (signed_progress < -runaway_threshold) {
      ax.brake_delta_ticks = progressed_ticks;
      ax.settle_start_ms = now_ms;
      ax.status_code = "ABORT_REVERSE_ENCODER";
      ax.phase = SPIN_SETTLING;
      setCoastFn();
      return;
    }

    // 3. Check for timeout / stall
    if ((now_ms - ax.start_ms) >= SPIN_REV_TIMEOUT_MS) {
      ax.brake_delta_ticks = progressed_ticks;
      ax.settle_start_ms = now_ms;
      ax.status_code = (llabs(progressed_ticks) < 10) ? "ABORT_STALLED_NO_TICKS" : "ABORT_TIMEOUT";
      ax.phase = SPIN_SETTLING;
      setCoastFn();
      return;
    }

    // 4. Proportional deceleration ramp in the final 35% of a revolution
    float slow_zone = max(60.0f, 0.35f * counts_per_rev);
    int cmd_mag = ax.max_pwm;
    if ((float)remaining < slow_zone) {
      float ratio = (float)remaining / slow_zone;
      cmd_mag = (int)roundf((float)min_approach_pwm + ratio * (float)(ax.max_pwm - min_approach_pwm));
      cmd_mag = constrain(cmd_mag, min_approach_pwm, ax.max_pwm);
    }
    setPwmFn(dir * cmd_mag);
  } else if (ax.phase == SPIN_BRAKING) {
    if ((now_ms - ax.brake_start_ms) >= BRAKE_HOLD_MS) {
      setCoastFn();
      ax.settle_start_ms = now_ms;
      ax.phase = SPIN_SETTLING;
    }
  } else if (ax.phase == SPIN_SETTLING) {
    if ((now_ms - ax.settle_start_ms) >= SETTLE_WAIT_MS) {
      ax.phase = SPIN_IDLE;
      emitRevResult(motor_label, ax, curr_ticks, curr_glitches, now_ms);
    }
  }
}

void printBanner() {
  Serial.println(F("# ================================================================"));
  Serial.println(F("# KENNY ESP32 Wheel Odometer, Diagnostics & 1-Rev Accuracy Firmware"));
  Serial.print(F("# Active Board Side : "));
  Serial.println(is_left_side ? F("LEFT (M1=34/25, M2=18/21)") : F("RIGHT (M1=34/25, M2=18/21)"));
  Serial.print(F("# Counts/Rev (CPR)  : "));
  Serial.println(counts_per_rev, 2);
  Serial.print(F("# Wheel Radius (m)  : "));
  Serial.println(wheel_radius_m, 4);
  Serial.println(F("# Commands          : ZERO | STOP | SIDE LEFT | SIDE RIGHT | CPR <n> | RADIUS <m>"));
  Serial.println(F("#                     INVERT <m1> <m2> | DEADBAND <pwm> | JOG <m1|m2|both> <pwm> [ms]"));
  Serial.println(F("#                     SPIN_REV <m1|m2|both> <revs> [max_pwm] | HELP"));
  Serial.println(F("# ================================================================"));
}

// ============================================================================
// Serial Command Parser
// ============================================================================
void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (*line == '\0') {
    return;
  }

  char* cmd = strtok(line, " \t");
  if (!cmd) {
    return;
  }

  if (strcasecmp(cmd, "HELP") == 0 || strcasecmp(cmd, "?") == 0) {
    printBanner();
  } else if (strcasecmp(cmd, "ZERO") == 0 || strcasecmp(cmd, "RESET") == 0 || strcasecmp(cmd, "Z") == 0) {
    stopMotors();
    zeroCounters();
    Serial.println(F("# OK ZEROED m1_ticks=0 m2_ticks=0"));
  } else if (strcasecmp(cmd, "STOP") == 0 || strcasecmp(cmd, "S") == 0) {
    stopMotors();
    Serial.println(F("# OK STOPPED"));
  } else if (strcasecmp(cmd, "SIDE") == 0) {
    char* arg = strtok(NULL, " \t");
    if (arg && strcasecmp(arg, "LEFT") == 0) {
      stopMotors();
      configureEncoderPins(true);
      Serial.println(F("# OK SIDE LEFT (M1=34/25, M2=18/21)"));
    } else if (arg && strcasecmp(arg, "RIGHT") == 0) {
      stopMotors();
      configureEncoderPins(false);
      Serial.println(F("# OK SIDE RIGHT (M1=34/25, M2=18/21)"));
    } else {
      Serial.println(F("# ERR Usage: SIDE LEFT | SIDE RIGHT"));
    }
  } else if (strcasecmp(cmd, "CPR") == 0) {
    char* arg = strtok(NULL, " \t");
    float val = arg ? atof(arg) : 0.0f;
    if (val > 0.0f) {
      counts_per_rev = val;
      Serial.print(F("# OK CPR "));
      Serial.println(counts_per_rev, 2);
    } else {
      Serial.println(F("# ERR Usage: CPR <positive_counts_per_rev>"));
    }
  } else if (strcasecmp(cmd, "RADIUS") == 0) {
    char* arg = strtok(NULL, " \t");
    float val = arg ? atof(arg) : 0.0f;
    if (val > 0.0f) {
      wheel_radius_m = val;
      Serial.print(F("# OK RADIUS "));
      Serial.println(wheel_radius_m, 4);
    } else {
      Serial.println(F("# ERR Usage: RADIUS <wheel_radius_meters>"));
    }
  } else if (strcasecmp(cmd, "DEADBAND") == 0) {
    char* arg = strtok(NULL, " \t");
    int val = arg ? atoi(arg) : 0;
    if (val >= 30 && val <= 200) {
      min_approach_pwm = val;
      Serial.print(F("# OK DEADBAND "));
      Serial.println(min_approach_pwm);
    } else {
      Serial.println(F("# ERR Usage: DEADBAND <30..200>"));
    }
  } else if (strcasecmp(cmd, "INVERT") == 0) {
    char* a1 = strtok(NULL, " \t");
    char* a2 = strtok(NULL, " \t");
    if (a1 && a2) {
      m1_sign = (atoi(a1) < 0) ? -1 : 1;
      m2_sign = (atoi(a2) < 0) ? -1 : 1;
      Serial.print(F("# OK INVERT m1_sign="));
      Serial.print(m1_sign);
      Serial.print(F(" m2_sign="));
      Serial.println(m2_sign);
    } else {
      Serial.println(F("# ERR Usage: INVERT <1|-1> <1|-1>"));
    }
  } else if (strcasecmp(cmd, "PULLUP") == 0) {
    char* arg = strtok(NULL, " \t");
    if (arg && strcasecmp(arg, "ON") == 0) {
      use_internal_pullups = true;
      configureEncoderPins(is_left_side);
      Serial.println(F("# OK PULLUP ON (GPIO34 remains external-only)"));
    } else if (arg && strcasecmp(arg, "OFF") == 0) {
      use_internal_pullups = false;
      configureEncoderPins(is_left_side);
      Serial.println(F("# OK PULLUP OFF"));
    } else {
      Serial.println(F("# ERR Usage: PULLUP ON | PULLUP OFF"));
    }
  } else if (strcasecmp(cmd, "JOG") == 0) {
    char* target = strtok(NULL, " \t");
    char* pwm_str = strtok(NULL, " \t");
    char* dur_str = strtok(NULL, " \t");
    if (!target || !pwm_str) {
      Serial.println(F("# ERR Usage: JOG <m1|m2|both> <pwm:-255..255> [duration_ms:1..5000]"));
      return;
    }
    stopMotors();
    int pwm = constrain(atoi(pwm_str), -255, 255);
    uint32_t dur_ms = dur_str ? (uint32_t)constrain(atoi(dur_str), 1, (int)MAX_JOG_DURATION_MS)
                              : DEFAULT_JOG_DURATION_MS;

    if (strcasecmp(target, "M1") == 0 || strcasecmp(target, "1") == 0) {
      setMotor1Pwm(pwm);
    } else if (strcasecmp(target, "M2") == 0 || strcasecmp(target, "2") == 0) {
      setMotor2Pwm(pwm);
    } else if (strcasecmp(target, "BOTH") == 0 || strcasecmp(target, "ALL") == 0) {
      setMotor1Pwm(pwm);
      setMotor2Pwm(pwm);
    } else {
      Serial.println(F("# ERR Target must be m1, m2, or both"));
      return;
    }

    jog_active = (current_m1_pwm != 0 || current_m2_pwm != 0);
    jog_deadline_ms = millis() + dur_ms;
    Serial.printf("# OK JOG target=%s pwm=%d duration_ms=%lu\n", target, pwm, (unsigned long)dur_ms);
  } else if (strcasecmp(cmd, "SPIN_REV") == 0) {
    char* target = strtok(NULL, " \t");
    char* rev_str = strtok(NULL, " \t");
    char* pwm_str = strtok(NULL, " \t");
    if (!target || !rev_str) {
      Serial.println(F("# ERR Usage: SPIN_REV <m1|m2|both> <revs> [max_pwm:60..255]"));
      return;
    }
    stopMotors();
    float revs = atof(rev_str);
    int max_pwm = pwm_str ? constrain(atoi(pwm_str), 60, 255) : 135;

    int64_t m1_t, m2_t;
    uint32_t m1_g, m2_g;
    noInterrupts();
    m1_t = isr_m1_ticks;
    m2_t = isr_m2_ticks;
    m1_g = isr_m1_glitches;
    m2_g = isr_m2_glitches;
    interrupts();

    uint32_t now_ms = millis();
    if (strcasecmp(target, "M1") == 0 || strcasecmp(target, "1") == 0) {
      startClosedLoopSpin(cl_m1, m1_t, m1_g, revs, max_pwm, now_ms);
    } else if (strcasecmp(target, "M2") == 0 || strcasecmp(target, "2") == 0) {
      startClosedLoopSpin(cl_m2, m2_t, m2_g, revs, max_pwm, now_ms);
    } else if (strcasecmp(target, "BOTH") == 0 || strcasecmp(target, "ALL") == 0) {
      startClosedLoopSpin(cl_m1, m1_t, m1_g, revs, max_pwm, now_ms);
      startClosedLoopSpin(cl_m2, m2_t, m2_g, revs, max_pwm, now_ms);
    } else {
      Serial.println(F("# ERR Target must be m1, m2, or both"));
      return;
    }
    Serial.printf("# OK SPIN_REV target=%s revs=%.4f max_pwm=%d\n", target, revs, max_pwm);
  } else {
    Serial.print(F("# ERR Unknown command: "));
    Serial.println(cmd);
  }
}

// ============================================================================
// Arduino Setup & Main Loop
// ============================================================================
void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  Serial.begin(SERIAL_BAUD);
  delay(200);

  configureEncoderPins(is_left_side);
  last_telemetry_ms = millis();
  printBanner();
}

void loop() {
  // 1. Read incoming serial commands
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      handleCommand(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len + 1 < sizeof(cmd_buf)) {
      cmd_buf[cmd_len++] = c;
    }
  }

  uint32_t now_ms = millis();

  // 2. Enforce open-loop motor jog timeout
  if (jog_active && (int32_t)(now_ms - jog_deadline_ms) >= 0) {
    stopMotors();
    Serial.println(F("# INFO JOG_FINISHED motors stopped"));
  }

  // 3. Run closed-loop 1-revolution spin state machine at full loop speed
  if (cl_m1.phase != SPIN_IDLE || cl_m2.phase != SPIN_IDLE) {
    int64_t m1_t, m2_t;
    uint32_t m1_g, m2_g;
    noInterrupts();
    m1_t = isr_m1_ticks;
    m2_t = isr_m2_ticks;
    m1_g = isr_m1_glitches;
    m2_g = isr_m2_glitches;
    interrupts();

    updateClosedLoopAxis(cl_m1, "M1", m1_t, m1_g, latest_m1_rpm, now_ms,
                         setMotor1Pwm, setMotor1ActiveBrake, setMotor1Coast);
    updateClosedLoopAxis(cl_m2, "M2", m2_t, m2_g, latest_m2_rpm, now_ms,
                         setMotor2Pwm, setMotor2ActiveBrake, setMotor2Coast);
  }

  // 4. Emit periodic odometer & diagnostic telemetry at 20 Hz
  uint32_t dt_ms = now_ms - last_telemetry_ms;
  if (dt_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now_ms;

    int64_t m1_ticks_snap, m2_ticks_snap;
    uint32_t m1_a_cnt, m1_b_cnt, m2_a_cnt, m2_b_cnt;
    uint32_t m1_err_snap, m2_err_snap;
    noInterrupts();
    m1_ticks_snap = isr_m1_ticks;
    m2_ticks_snap = isr_m2_ticks;
    m1_a_cnt = isr_m1_a_edges;
    m1_b_cnt = isr_m1_b_edges;
    m2_a_cnt = isr_m2_a_edges;
    m2_b_cnt = isr_m2_b_edges;
    m1_err_snap = isr_m1_glitches;
    m2_err_snap = isr_m2_glitches;
    interrupts();

    int64_t m1_dt = m1_ticks_snap - prev_m1_ticks;
    int64_t m2_dt = m2_ticks_snap - prev_m2_ticks;
    prev_m1_ticks = m1_ticks_snap;
    prev_m2_ticks = m2_ticks_snap;

    float dt_s = (float)dt_ms * 0.001f;
    float circumference_m = 2.0f * (float)M_PI * wheel_radius_m;
    float m1_revs = (float)m1_ticks_snap / counts_per_rev;
    float m2_revs = (float)m2_ticks_snap / counts_per_rev;
    float m1_dist_m = m1_revs * circumference_m;
    float m2_dist_m = m2_revs * circumference_m;
    float m1_rps = ((float)m1_dt / counts_per_rev) / dt_s;
    float m2_rps = ((float)m2_dt / counts_per_rev) / dt_s;
    float m1_mps = m1_rps * circumference_m;
    float m2_mps = m2_rps * circumference_m;
    latest_m1_rpm = m1_rps * 60.0f;
    latest_m2_rpm = m2_rps * 60.0f;

    uint8_t m1_a = (uint8_t)digitalRead(pin_m1_a);
    uint8_t m1_b = (uint8_t)digitalRead(pin_m1_b);
    uint8_t m2_a = (uint8_t)digitalRead(pin_m2_a);
    uint8_t m2_b = (uint8_t)digitalRead(pin_m2_b);

    Serial.printf(
        "ODOM side=%s ms=%lu m1_ticks=%lld m2_ticks=%lld m1_dt=%lld m2_dt=%lld "
        "m1_revs=%.4f m2_revs=%.4f m1_m=%.4f m2_m=%.4f m1_mps=%.4f m2_mps=%.4f "
        "m1_pwm=%d m2_pwm=%d m1_ab=%u%u m2_ab=%u%u m1_err=%lu m2_err=%lu "
        "m1_a_edges=%lu m1_b_edges=%lu m2_a_edges=%lu m2_b_edges=%lu\n",
        is_left_side ? "LEFT" : "RIGHT",
        (unsigned long)now_ms,
        (long long)m1_ticks_snap,
        (long long)m2_ticks_snap,
        (long long)m1_dt,
        (long long)m2_dt,
        m1_revs,
        m2_revs,
        m1_dist_m,
        m2_dist_m,
        m1_mps,
        m2_mps,
        current_m1_pwm,
        current_m2_pwm,
        m1_a, m1_b,
        m2_a, m2_b,
        (unsigned long)m1_err_snap,
        (unsigned long)m2_err_snap,
        (unsigned long)m1_a_cnt,
        (unsigned long)m1_b_cnt,
        (unsigned long)m2_a_cnt,
        (unsigned long)m2_b_cnt);
  }
}
