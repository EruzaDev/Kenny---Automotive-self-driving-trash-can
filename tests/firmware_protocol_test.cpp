#include "../firmware/kenny_esp32/protocol.h"
#include <cassert>
#include <iostream>
#include <random>
#include <string>

int main() {
  using namespace kenny;
  assert(parse("HELLO").kind == Kind::Hello);
  assert(parse(" STATUS\t\r").kind == Kind::Status);
  assert(parse("STOP").kind == Kind::Stop);
  assert(parse("ARM").kind == Kind::Arm);
  auto config = parse("CONFIG 1320 1440 1320 1440");
  assert(config.kind == Kind::Configure && config.counts[1] == 1440);
  auto velocity = parse("V 4294967295 0.1 -0.6");
  assert(velocity.kind == Kind::Velocity && velocity.sequence == UINT32_MAX);
  for (const char* line : {"", "ARM extra", "CONFIG 0 1 1 1", "CONFIG 1 1 1",
                           "CONFIG -1 1 1 1", "CONFIG 1000001 1 1 1", "V 1 nan 0",
                           "V 1 inf 0", "V 1 0 -inf", "V 1 -0.01 0", "V 1 0.11 0",
                           "V 1 0 0.61", "V -1 0 0", "V 4294967296 0 0", "V 1 0 0 extra",
                           "V 1 1e100 0", "V 1 0.05junk 0"}) {
    assert(parse(line).kind == Kind::Invalid);
  }
  assert(parse(std::string(10000, 'x').c_str()).kind == Kind::Invalid);
  assert(newer(2, 1) && newer(0, UINT32_MAX));
  assert(!newer(1, 1) && !newer(1, 2) && !newer(0x80000000U, 0));
  assert(!timedOut(199, 0) && timedOut(200, 0));
  assert(!timedOut(49, UINT32_MAX - 100) && timedOut(100, UINT32_MAX - 100));
  // Arbitrary serial input must stay bounded and never admit nonfinite/out-of-range motion.
  std::mt19937 random(42);
  for (unsigned test = 0; test < 10000; ++test) {
    std::string line;
    for (unsigned length = random() % 160; length; --length) line += char(1 + random() % 126);
    Command command = parse(line.c_str());
    if (command.kind == Kind::Velocity) {
      assert(std::isfinite(command.linear) && std::isfinite(command.angular));
      assert(command.linear >= 0 && command.linear <= MAX_LINEAR_MPS);
      assert(std::fabs(command.angular) <= MAX_TURN_RPS);
    }
  }
  std::cout << "Firmware protocol, numeric bounds, sequence wrap, watchdog wrap and 10000 fuzz cases passed\n";
}
