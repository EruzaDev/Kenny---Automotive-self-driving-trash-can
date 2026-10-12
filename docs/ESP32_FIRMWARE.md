# ESP32-WROOM: four JGB37-520 motors and two L298N boards

The Arduino sketch is `firmware/kenny_esp32/kenny_esp32.ino` with its companion
`protocol.h` in the same folder. This is a **four-wheel skid-steer commissioning
controller**, not the completed autonomous robot stack. The original simulator
assumed two driven wheels; four-wheel traction, effective turning track and slip
need measurement before transferring a policy.

## Hardware required

- Classic **ESP32 DevKit / ESP32-WROOM-32**, with the GPIOs below exposed.
- **Two L298N boards**, one channel per motor. This design does not parallel two
  motors on one channel. One L298N board is insufficient for this wiring plan.
- Four JGB37-520 gearmotors **with quadrature Hall encoders**. If yours have only
  two motor wires and no encoder, this closed-loop firmware cannot operate them.
- Motor supply matching the exact voltage variant, fused wiring, and a physical
  emergency stop that cuts motor power or reliably disables both drivers.
- Proper encoder level conversion/pull-ups, and four 10 kΩ PWM-enable pull-downs.

**JGB37-520 is a family, not one electrical specification.** Record rated voltage,
loaded current and stall current. L298N modules have substantial voltage drop
and heat loss and may be unsuitable for these motors. Use a driver with verified
continuous/peak capability if the motor demand exceeds the module's real rating;
the advertised "2 A" figure is not proof of acceptable operation. Firmware PWM
limits do not provide current limiting or make an undersized driver suitable.

Do not choose wiring by generic JGB37-520 wire colors: obtain the exact motor's
pinout. Identify **M+/M−, encoder VCC/GND, encoder A/B** before connection.

### Confirmed supplier pinout: NFP-JGB37-520-EN, 333 RPM

The owner supplied [this Precision Mini Drives product page](https://precisionminidrives.com/product/37mm-gear-motor-with-encoder-41mm-type-model-nfp-jgb37-520-en).
Its [encoder wiring diagram](https://precisionminidrives.com/wp-content/uploads/2023/01/NFP-JGB37-520-EN-gear-motor-encoder-wiring-tips.png)
identifies these colors and specifies encoder supply **DC 3.3 V / DC 5.0 V**:

| Color | Function | Connection in this design |
| --- | --- | --- |
| Red | Motor winding positive | First assigned L298N OUT terminal |
| White | Motor winding negative | Second assigned L298N OUT terminal |
| Black | Encoder ground | Common GND |
| Blue | Encoder supply positive | ESP32 3V3 / suitable regulated 3.3 V supply |
| Yellow | C1 / encoder A | Assigned encoder A GPIO |
| Green | C2 / encoder B | Assigned encoder B GPIO |

Use the supported **3.3 V encoder supply** for ESP32-level signals and keep it
stable under all four encoder loads. The diagram states built-in pull-up shaping
resistors, so this supplier's encoder is not assumed to need external A/B pull-ups.
Do not power blue from 12 V. Verify the received motors match this supplied model.

The **333 RPM** row is model **NFP-JGB37-520-EN-12100**, **12 V**, **30:1**:
333 RPM no-load, 250 RPM rated load, 0.65 A rated-load current and **2.4 A stall
current per motor**. That stall demand exceeds the common L298N continuous-channel
rating. Do not treat the existing L298N boards as suitable for unrestricted loaded
robot use; choose drivers with verified continuous/peak ratings and thermal/current
protection appropriate to this motor. Four simultaneously stalled motors would
theoretically demand 9.6 A at the specified conditions. Do not deliberately stall
them to test it; the supplier explicitly forbids locking the motor.

## Wiring table

Driver **F** operates the front motors. Driver **R** operates the rear motors.
Channel A is left; channel B is right on both boards.

| ESP32 GPIO | Connection |
| --- | --- |
| 16 | F IN1: left-front direction |
| 17 | F IN2: left-front direction |
| 18 | F IN3: right-front direction |
| 19 | F IN4: right-front direction |
| 21 | R IN1: left-rear direction |
| 22 | R IN2: left-rear direction |
| 4 | R IN3: right-rear direction |
| 5 | R IN4: right-rear direction |
| 25 | F ENA: left-front PWM |
| 26 | F ENB: right-front PWM |
| 32 | R ENA: left-rear PWM |
| 33 | R ENB: right-rear PWM |
| 34 | Left-front encoder A |
| 27 | Left-front encoder B |
| 35 | Right-front encoder A |
| 14 | Right-front encoder B |
| 36 / VP | Left-rear encoder A |
| 13 | Left-rear encoder B |
| 39 / VN | Right-rear encoder A |
| 23 | Right-rear encoder B |
| 15 | Normally-closed E-stop auxiliary contact; other contact terminal → GND |
| GND | Common logic ground: ESP32, both drivers, encoders, motor-supply return |

Some 30-pin DevKit variants do not expose GPIO36/39; check your board before
following this table. This mapping is for WROOM, not WROVER/S3/C3 or arbitrary
ESP32 variants. UART0 GPIO1/3 remain reserved for the board's USB serial bridge.

All four channels have their **own** direction and PWM signals. There are no
shared IN wires between boards. GPIO5 and GPIO15 are boot-strapping pins: do not
add circuitry that forces GPIO5 low at reset; a 10 kΩ pull-up to 3.3 V preserves
its default boot level. The normally-closed E-stop contact pulls GPIO15 low,
which suppresses the ROM boot messages on classic ESP32 but still permits normal
flash boot. PWM-enable pull-downs keep motors disabled during boot. Verify these
pins against your exact DevKit's onboard circuitry before wiring.

Turning is skid steering: for a left turn, the right-side wheel targets are
faster than the left-side targets; for an in-place left turn, the left wheels
run backward and right wheels forward. Each motor has independent hardware
outputs and its own encoder feedback even when same-side speed targets match.

Motor terminals:

| Driver | OUT1/OUT2 | OUT3/OUT4 |
| --- | --- | --- |
| Front F | Left-front M+/M− | Right-front M+/M− |
| Rear R | Left-rear M+/M− | Right-rear M+/M− |

1. **Remove all four ENA/ENB jumpers.** Connect each enable to its assigned PWM
   pin and a **10 kΩ pull-down to GND**, so drive stays disabled during reset.
2. Connect each IN pin only to its assigned GPIO in the table. Do not join IN
   pins or motor output terminals between boards.
3. Wire motor polarity so a positive left/right command moves every wheel forward
   on the chassis. Opposite-side mirrored mounting often requires reversed motor
   polarity. Swap that motor's M+/M− if needed; do not infer from wire color.
4. Encoder A/B signals must never exceed **3.3 V at an ESP32 GPIO**. If encoders
   require 5 V supply, use suitable 5 V-to-3.3 V signal conversion. For confirmed
   open-collector outputs, use external pull-ups to 3.3 V as allowed by the motor
   encoder specification. GPIO34/35/36/39 have **no internal pull-ups**. The sketch
   uses INPUT mode for all eight encoder signals; do not leave them floating.
5. Feed the motors from their proper supply, not from the ESP32 or Pi. Configure
   each L298N module's separate 5 V logic supply/regulator jumper according to that
   exact module. Do not connect an enabled onboard regulator's output to another
   5 V source, or use it to back-power the Pi/USB-powered ESP32.
6. Connect the ESP32 to the Pi with a USB **data** cable. No Pi GPIO UART wiring
   is needed. Verify the ESP32 serial port independently of the LiDAR adapter.
7. GPIO15 expects a normally-closed auxiliary switch contact to ground: HIGH/open
   means stop. It supplements the hardware motor-power stop; it is not a substitute.
   A commissioning jumper from GPIO15 to GND is only for a guarded wheels-raised
   bench test, not normal robot operation.

## Flash from Arduino IDE

1. Install **esp32 by Espressif Systems**, version **3.3.12** (the compiled version).
   Add `https://espressif.github.io/arduino-esp32/package_esp32_index.json` to the
   IDE's Additional Boards Manager URLs if necessary.
2. Open `firmware/kenny_esp32/kenny_esp32.ino`. Keep `protocol.h` beside it; the IDE
   shows it as another tab.
3. Select **ESP32 Dev Module**, the correct USB port, and upload. Keep motor power
   disconnected for the initial flash.
4. Open Serial Monitor at **115200 baud**, line ending **Newline**.
5. Expect `KENNY_ESP32_L298N_V1 READY_DISABLED` followed by telemetry.

The sketch uses Arduino-ESP32 **3.x** `ledcAttach`/pin-based `ledcWrite`; it will
not compile unchanged against the old 2.x LEDC API.

### Compile/upload from the Pi

Arduino CLI 1.5.1 and the ESP32 3.3.12 core were installed for compile verification.
The CLI binary currently lives in `/tmp/opencode/arduino-cli`; install the official
ARM64 CLI somewhere persistent if you need it after temporary-directory cleanup.

```bash
cd ~/Kenny---Automotive-self-driving-trash-can
/tmp/opencode/arduino-cli compile --fqbn esp32:esp32:esp32 \
  --jobs 2 --warnings all firmware/kenny_esp32
/tmp/opencode/arduino-cli board list
# Substitute the verified ESP32 port, never the LiDAR port:
/tmp/opencode/arduino-cli upload --fqbn esp32:esp32:esp32 \
  --port /dev/serial/by-id/YOUR_ESP32_DEVICE firmware/kenny_esp32
```

## Encoder configuration: expected 1,320 x4 counts per wheel revolution

The owner initially specified 660 counts/revolution. The supplied motor page
subsequently identified **11 pulses per motor-shaft revolution** and a **30:1**
gearbox for the 333 RPM version. The expected output-wheel counts are:

- x1 (one channel, one edge): `11 × 30 = 330`.
- x2 (two transitions per cycle): `11 × 30 × 2 = 660`.
- **x4 (this sketch, both edges on both channels): `11 × 30 × 4 = 1320`.**

The firmware default is now `DEFAULT_ENCODER_CPR = 1320`. Confirm it with the
actual received motors before powered testing. If the sketch itself measures
660 counts for one full output revolution, use that observed value instead;
do not confuse a different decoding convention with a physical calibration.

Motors remain disarmed while measuring counts; motor power can remain disconnected.
The decoder counts every valid quadrature transition (**x4 decoding**) at the
gearbox output wheel. Do not use a guessed pulses/revolution value from a listing.

1. Send `STATUS` and record the four counts, ordered **LF RF LR RR**.
2. Slowly rotate one wheel **forward** exactly ten complete revolutions by hand.
3. Send `STATUS` again. The absolute count difference divided by ten is that
   wheel's **counts per wheel revolution**. Repeat for all four wheels.
4. Counts should increase for forward rotation. If a motor's count decreases,
   change that motor's `encoderSign` from `+1` to `-1` in the `motors` array and
   reflash. Verify again. Do not fix a count-sign error by guessing PID gains.
5. Supply the four measured integer values with:

```text
CONFIG <LF_CPR> <RF_CPR> <LR_CPR> <RR_CPR>
```

Replace the placeholders; do not type the angle brackets. For the supplied values:

```text
CONFIG 1320 1320 1320 1320
```

CONFIG always stops/disarms the motors. Overrides are held in RAM; restarting
restores the expected default 1320 for each motor. Motors remain disabled until ARM, even
with CPR defaults set. Set the default to zero if you want to require explicit
CONFIG before every ARM instead.

## USB serial protocol

ASCII lines, 115200 baud, 8-N-1, maximum 95 characters, newline terminated.

| Command | Meaning |
| --- | --- |
| `HELLO` | Identify firmware/protocol; does not enable motion |
| `STATUS` | Return an immediate telemetry line |
| `CONFIG LF RF LR RR` | Set measured x4 encoder CPR; stop/disarm |
| `ARM` | Arm at zero targets if CPR and E-stop contact are valid |
| `V sequence linear angular` | Forward m/s and turn rad/s; increasing uint32 sequence |
| `STOP` | Immediately remove drive command and disarm |

Examples of valid velocity messages:

```text
V 1 0.00 0.00
V 2 0.03 0.00
V 3 0.00 0.20
```

Send fresh V messages at **20 Hz** (every 50 ms). ARM starts a **200 ms watchdog**;
if no valid fresh V arrives, the firmware zeros PWM and disarms. A manually typed
ARM followed much later by a Serial Monitor V will therefore be rejected. Use a
Pi process to stream commands for motion testing; Serial Monitor is appropriate
for HELLO, STATUS and encoder calibration. No motion sender or ROS motor bridge
is automatically started by this firmware setup.

Linear requests are limited to **0–0.10 m/s**, turn requests to **±0.60 rad/s**,
and each wheel to **0.15 m/s** by rescaling both sides together. A turn can reverse
one side's wheels even though straight reverse travel is disabled. Maximum PWM
is **90/255**; these are commissioning limits, not measured braking/current limits.

Telemetry at up to 10 Hz:

```text
S milliseconds last_sequence armed fault_bits countLF countRF countLR countRR rpmLF rpmRF rpmLR rpmRR
```

`last_sequence` is meaningful for the current ARM session only after the first
accepted V message. Fault bits: **1 watchdog**, **2 E-stop**, **4 malformed command**,
**8 missing/opposite-direction encoder motion**, **16 missing CPR configuration**.
Faults are latched until a deliberate CONFIG/ARM; ARM still requires a valid
E-stop contact and all CPR values. Invalid/nonfinite/out-of-bounds motion input
stops and disarms. Replayed/out-of-order sequences do not refresh the watchdog.

## First powered test and tuning

With all four wheels raised, verify forward motor polarity and positive encoder
sign, then test STOP, E-stop, serial disconnection and a crashed command sender.
`KP=1.5`, `KI=0.5`, nominal 333 RPM feedforward and the 90 PWM cap are untuned
starting values. The controller may stop with an encoder fault if a motor cannot
overcome friction at that cap; investigate power, wiring, counts and driver sizing
before changing limits. The 500 ms no-pulse/direction monitor is a diagnostic,
not an overcurrent protector.

Zero PWM with enables low **does not guarantee active braking**; the wheels may
coast. Measure stopping distance and loaded four-wheel turning behavior. The
0.235 m track and 0.04 m radius are provisional; skid steering often needs a
calibrated effective track rather than geometric spacing alone.

This firmware implements only base control. Cliff interlocks, actual E-stop power
hardware, the Pi's guarded `/cmd_vel/safe` bridge and four-wheel odometry/localization
must be integrated before autonomous navigation. Sensor GUI rays are visualization
and are not a motor safety controller.

## Software validation

The sketch is compiled for `esp32:esp32:esp32` with Arduino-ESP32 3.3.12. Host-side
protocol tests check command bounds, invalid/NaN input, sequence/watchdog counter
wrap and 10,000 randomized input lines:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  tests/firmware_protocol_test.cpp -o /tmp/kenny-firmware-protocol-test
/tmp/kenny-firmware-protocol-test
```

Compilation and parser tests do not verify physical pin wiring, encoder electrical
levels, current capacity, motor polarity or PID behavior. No ESP32 was connected
for flashing or physical motion tests when this firmware was prepared.
