#include "../firmware/kenny_mobile/control.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <random>

mobile::Command arm(uint32_t now, uint32_t lease = 123) {
  mobile::Command command;
  command.kind = mobile::Kind::Arm; command.receivedMs = now; command.lease = lease;
  return command;
}
mobile::Command drive(uint32_t now, uint32_t sequence, float linear = 0.05f, float angular = 0) {
  mobile::Command command;
  command.kind = mobile::Kind::Drive; command.receivedMs = now;
  command.lease = 123; command.sequence = sequence; command.linear = linear; command.angular = angular;
  return command;
}

int main() {
  using namespace mobile;
  uint32_t number;
  float value;
  assert(readUnsigned("4294967295", number) && number == UINT32_MAX);
  for (const char* text : {"", "-1", "4294967296", "12junk"}) assert(!readUnsigned(text, number));
  for (const char* text : {"", "nan", "inf", "1e100", "0.05junk"}) assert(!readFloat(text, value));
  assert(readFloat("-0.05", value) && value < 0);

  State s;
  assert(!s.armed && s.linear == 0 && s.angular == 0);
  assert(!s.apply(drive(0, 1), 0, false));
  assert(!s.apply(arm(0), 0, true));
  assert(s.faults & ESTOP);
  assert(!s.apply(arm(0), 0, false, false));
  assert(!s.apply(arm(0), 100, false));  // Old queued ARM cannot activate drive.
  assert(s.apply(arm(100), 100, false));
  assert(s.armed && s.faults == 0 && s.linear == 0);
  assert(!s.apply(arm(110, 456), 110, false));  // No takeover of an active controller.
  auto other = drive(120, 1); other.lease = 456;
  assert(!s.apply(other, 120, false));
  assert(s.apply(drive(130, 10, -0.05f, 0.2f), 130, false));
  assert(s.linear < 0 && s.angular > 0);
  assert(!s.apply(drive(140, 10), 140, false));
  assert(!s.apply(drive(140, 9), 140, false));
  assert(s.lastCommandMs == 130);  // Replays don't extend watchdog.
  s.tick(429, false); assert(s.armed);
  s.tick(430, false); assert(!s.armed && s.lease == 0 && s.linear == 0 && (s.faults & WATCHDOG));
  assert(!s.apply(drive(431, 11), 431, false));  // Late packets cannot re-arm.
  assert(s.apply(arm(450), 450, false));
  assert(!s.apply(drive(460, 1, 0.101f), 460, false));
  assert(!s.armed && (s.faults & BAD_COMMAND));
  assert(s.apply(arm(470), 470, false));
  assert(!s.apply(drive(480, 1, std::numeric_limits<float>::quiet_NaN()), 480, false));
  assert(!s.armed);
  assert(s.apply(arm(490), 490, false));
  assert(s.apply(drive(500, 1), 500, false));
  assert(s.apply(drive(510, 2, 0, 0), 510, false));  // Release keeps session, exact zero targets.
  assert(s.armed && s.linear == 0 && s.angular == 0);
  s.tick(511, true); assert(!s.armed && (s.faults & ESTOP));
  Command stop;
  assert(s.apply(stop, 100000, true));  // STOP accepted even when old/not armed.

  State wrap;
  assert(wrap.apply(arm(UINT32_MAX - 100), UINT32_MAX - 100, false));
  assert(wrap.apply(drive(UINT32_MAX - 90, UINT32_MAX), UINT32_MAX - 90, false));
  assert(wrap.apply(drive(10, 0), 10, false));
  wrap.tick(309, false); assert(wrap.armed);
  wrap.tick(310, false); assert(!wrap.armed);

  float left, right;
  wheelSpeeds(0.05f, 0, left, right); assert(left == right && left > 0);
  wheelSpeeds(0, 0.5f, left, right); assert(left < 0 && right > 0);
  wheelSpeeds(0, -0.5f, left, right); assert(left > 0 && right < 0);
  wheelSpeeds(-0.05f, 0, left, right); assert(left == right && left < 0);
  std::mt19937 random(42);
  for (unsigned i = 0; i < 10000; ++i) {
    float v = (int(random()%2001) - 1000) / 10000.0f;
    float w = (int(random()%1201) - 600) / 1000.0f;
    wheelSpeeds(v, w, left, right);
    assert(std::fabs(left) <= MAX_WHEEL + 1e-6f && std::fabs(right) <= MAX_WHEEL + 1e-6f);
  }
  std::cout << "Mobile control checks passed: ownership, bounds/NaN, release, stop, E-stop, stale queue, replay, counter wraps and 10000 wheel-limit cases\n";
}
