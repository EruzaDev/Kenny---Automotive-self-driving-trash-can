#pragma once
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace kenny {
constexpr unsigned LINE_CAPACITY = 96;
constexpr uint32_t WATCHDOG_MS = 200;
constexpr float MAX_LINEAR_MPS = 0.10f;
constexpr float MAX_TURN_RPS = 0.60f;
enum class Kind { Invalid, Hello, Status, Configure, Arm, Stop, Velocity };
struct Command {
  Kind kind = Kind::Invalid;
  uint32_t sequence = 0;
  float linear = 0;
  float angular = 0;
  uint32_t counts[4] = {};
};

inline bool unsignedValue(const char* text, uint32_t& result) {
  if (!text || *text < '0' || *text > '9') return false;
  errno = 0;
  char* end = nullptr;
  unsigned long long value = std::strtoull(text, &end, 10);
  if (errno || *end || value > UINT32_MAX) return false;
  result = static_cast<uint32_t>(value);
  return true;
}

inline bool floatValue(const char* text, float& result) {
  errno = 0;
  char* end = nullptr;
  result = std::strtof(text, &end);
  return end != text && !*end && !errno && std::isfinite(result);
}

inline Command parse(const char* line) {
  Command command;
  if (std::strlen(line) >= LINE_CAPACITY) return command;
  char copy[LINE_CAPACITY];
  std::strcpy(copy, line);
  char* tokens[6] = {};
  unsigned count = 0;
  char* cursor = copy;
  while (*cursor) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r') ++cursor;
    if (!*cursor) break;
    if (count == 6) return command;
    tokens[count++] = cursor;
    while (*cursor && *cursor != ' ' && *cursor != '\t' && *cursor != '\r') ++cursor;
    if (*cursor) *cursor++ = '\0';
  }
  if (!count) return command;
  if (count == 1) {
    if (!std::strcmp(tokens[0], "HELLO")) command.kind = Kind::Hello;
    if (!std::strcmp(tokens[0], "STATUS")) command.kind = Kind::Status;
    if (!std::strcmp(tokens[0], "ARM")) command.kind = Kind::Arm;
    if (!std::strcmp(tokens[0], "STOP")) command.kind = Kind::Stop;
  } else if (count == 5 && !std::strcmp(tokens[0], "CONFIG")) {
    for (unsigned i = 0; i < 4; ++i) {
      if (!unsignedValue(tokens[i + 1], command.counts[i]) || !command.counts[i] ||
          command.counts[i] > 1000000) return Command{};
    }
    command.kind = Kind::Configure;
  } else if (count == 4 && !std::strcmp(tokens[0], "V")) {
    if (!unsignedValue(tokens[1], command.sequence) ||
        !floatValue(tokens[2], command.linear) || !floatValue(tokens[3], command.angular) ||
        command.linear < 0 || command.linear > MAX_LINEAR_MPS ||
        std::fabs(command.angular) > MAX_TURN_RPS) return Command{};
    command.kind = Kind::Velocity;
  }
  return command;
}

inline bool newer(uint32_t sequence, uint32_t previous) {
  uint32_t difference = sequence - previous;
  return difference != 0 && difference < 0x80000000UL;
}

inline bool timedOut(uint32_t now, uint32_t previous) {
  return static_cast<uint32_t>(now - previous) >= WATCHDOG_MS;
}
}  // namespace kenny
