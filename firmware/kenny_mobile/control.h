#pragma once
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace mobile {
constexpr uint32_t WATCHDOG_MS = 300;
constexpr uint32_t QUEUE_MAX_AGE_MS = 100;
constexpr float MAX_LINEAR = 0.10f;
constexpr float MAX_ANGULAR = 0.60f;
constexpr float MAX_WHEEL = 0.15f;
constexpr float TRACK = 0.235f;
enum Fault : uint32_t { WATCHDOG = 1, ESTOP = 2, BAD_COMMAND = 4, ENCODER = 8,
                        TIMING = 16, CONFIGURATION = 32, PWM_INIT = 64, I2C_DRIVER = 128 };
enum class Kind { Arm, Drive, Stop };
struct Command {
  Kind kind = Kind::Stop;
  uint32_t id = 0, lease = 0, sequence = 0, receivedMs = 0;
  float linear = 0, angular = 0;
};
inline bool readUnsigned(const char* text, uint32_t& value) {
  if (!text || *text < '0' || *text > '9') return false;
  errno = 0;
  char* end = nullptr;
  unsigned long long parsed = std::strtoull(text, &end, 10);
  if (errno || *end || parsed > UINT32_MAX) return false;
  value = static_cast<uint32_t>(parsed);
  return true;
}
inline bool readFloat(const char* text, float& value) {
  if (!text || !*text) return false;
  errno = 0;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && !*end && !errno && std::isfinite(value);
}
inline bool newer(uint32_t value, uint32_t previous) {
  uint32_t delta = value - previous;
  return delta && delta < 0x80000000UL;
}
inline void wheelSpeeds(float linear, float angular, float& left, float& right) {
  left = linear - angular * TRACK / 2;
  right = linear + angular * TRACK / 2;
  float peak = std::fmax(std::fabs(left), std::fabs(right));
  if (peak > MAX_WHEEL) { left *= MAX_WHEEL / peak; right *= MAX_WHEEL / peak; }
}

// Only the motor task owns this state. HTTP handlers exchange bounded messages.
struct State {
  bool armed = false, hasSequence = false;
  uint32_t lease = 0, sequence = 0, lastCommandMs = 0, faults = 0;
  float linear = 0, angular = 0;
  void stop(uint32_t reason = 0) {
    armed = false; lease = 0; linear = angular = 0; faults |= reason;
  }
  void tick(uint32_t now, bool estop) {
    if (estop) stop(ESTOP);
    if (armed && static_cast<uint32_t>(now - lastCommandMs) >= WATCHDOG_MS) stop(WATCHDOG);
  }
  bool apply(const Command& command, uint32_t now, bool estop, bool ready = true) {
    tick(now, estop);
    if (command.kind == Kind::Stop) { stop(); return true; }
    if (static_cast<uint32_t>(now - command.receivedMs) >= QUEUE_MAX_AGE_MS) return false;
    if (command.kind == Kind::Arm) {
      if (armed || !command.lease || estop || !ready) return false;
      armed = true; lease = command.lease; hasSequence = false;
      linear = angular = 0; lastCommandMs = now; faults = 0;
      return true;
    }
    if (!armed || !command.lease || command.lease != lease) return false;
    if (!std::isfinite(command.linear) || !std::isfinite(command.angular) ||
        std::fabs(command.linear) > MAX_LINEAR || std::fabs(command.angular) > MAX_ANGULAR) {
      stop(BAD_COMMAND); return false;
    }
    if (hasSequence && !newer(command.sequence, sequence)) return false;
    hasSequence = true; sequence = command.sequence; lastCommandMs = now;
    linear = command.linear; angular = command.angular;
    return true;
  }
};
}  // namespace mobile
