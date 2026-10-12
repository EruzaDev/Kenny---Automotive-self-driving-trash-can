# KENNY ESP32 Wi-Fi joystick

The ESP32 hosts its own Wi-Fi network and self-contained phone web page. No phone
app, internet connection, Pi process or extra Arduino library is required after
flashing. This is **manual commissioning control**, not autonomous navigation.

## Select the wiring before upload

Open `firmware/kenny_mobile/config.h`. Use **one** of these settings:

```cpp
// Earlier independent ESP32-to-L298N wiring (shipped default):
#define KENNY_WIRING KENNY_DIRECT_GPIO

// Replace the line above with this for the new carrier PCB:
#define KENNY_WIRING KENNY_CARRIER_MCP23017
```

The modes have different direction and E-stop connections. The phone page and
USB startup message identify the selected mode. Do not upload direct mode onto
the carrier PCB: GPIO16/17/21/22 have different purposes there.

### Direct-GPIO mode: classic 38-pin ESP32-WROOM

Numbers below are printed **GPIO labels**, not physical header positions.

| Motor | External driver | Red / white | Direction terminals → GPIO | Enable → GPIO | Yellow A / green B → GPIO |
| --- | --- | --- | --- | --- | --- |
| Left-front | L298N #1 A | OUT1 / OUT2 | IN1 → 16, IN2 → 17 | ENA → 25 | 34 / 27 |
| Right-front | L298N #1 B | OUT3 / OUT4 | IN3 → 18, IN4 → 19 | ENB → 26 | 35 / 14 |
| Left-rear | L298N #2 A | OUT1 / OUT2 | IN1 → 21, IN2 → 22 | ENA → 32 | 36 (VP) / 13 |
| Right-rear | L298N #2 B | OUT3 / OUT4 | IN3 → 4, IN4 → 5 | ENB → 33 | 39 (VN) / 23 |

All encoder **blue → stable 3.3 V**, **black → common GND**, as permitted by the
[identified supplier](https://precisionminidrives.com/product/37mm-gear-motor-with-encoder-41mm-type-model-nfp-jgb37-520-en).
GPIO34/35/36/39 have no internal pull-ups; this encoder has output pull-up circuitry.

- Remove all ENA/ENB jumpers; use **10 kΩ enable-to-GND pull-downs**.
- Preserve GPIO5's default boot HIGH level with the documented **10 kΩ pull-up
  to 3.3 V**. PWM pull-downs keep motors disabled during reset.
- E-stop input: **GPIO15 → normally-closed auxiliary contact → GND**. Open=stop.
  GPIO15 LOW suppresses ROM boot messages on classic ESP32 but permits flash boot.
- Power motors from their appropriate fused/switched **12 V motor supply**,
  never from the ESP32/Pi. Join ESP32, encoder, driver and supply-return grounds.
- Configure L298N logic-supply/regulator jumpers according to the exact modules;
  do not back-power USB/ESP32. The physical motor-power E-stop is separate.
- This mapping is for **WROOM**, not WROVER/S3/C3. GPIO16/17/21/22 are used for
  motor directions, so do not add the carrier sensor connections in direct mode.

### Carrier-PCB mode

Use the terminal blocks in [`hardware/kenny_carrier/README.md`](../hardware/kenny_carrier/README.md):

- Direct PWM: **LF 25, RF 26, LR 32, RR 33**; encoder GPIOs unchanged.
- E-stop: **J9 → GPIO18**, not GPIO15.
- I²C **SDA21/SCL22**, 100 kHz, MCP23017 at **0x20**.
- Directions: **GPB0/1 LF, GPB2/3 RF, GPB4/5 LR, GPB6/7 RR**.
- HX711 GPIO16/17 remain reserved; SCK is held LOW. This sketch controls motors
  and reports encoders, but **does not yet read MPU6050 or load-cell measurements**.

The expander's latch is zeroed before its direction outputs are enabled. Unused
port A remains input with pull-ups. I²C transactions have a 10 ms timeout, and
direction mode/latch are checked approximately every 100 ms. An I²C error or
unexpected expander reset zeros all direct PWM enables and disarms. Fix the
connection and **restart the ESP32**; recovery does not automatically re-arm.

## Upload

Open **`firmware/kenny_mobile/kenny_mobile.ino`** in Arduino IDE. Keep **config.h,
control.h and web_ui.h** beside it. Install **esp32 by Espressif Systems 3.3.12**,
select **ESP32 Dev Module**, select the verified ESP32 USB port, choose the wiring,
and upload initially with motor power disconnected.

WiFi, WebServer and Wire are included in the ESP32 core. The sketch uses the
Arduino-ESP32 **3.x** LEDC API. Serial Monitor at **115200 baud** shows the mode
and URL. USB serial is monitoring only; the earlier serial ARM/V protocol is
not active in this mobile sketch.

### CLI on this Pi

Arduino CLI is installed persistently at `~/.local/bin/arduino-cli`:

```bash
cd ~/Kenny---Automotive-self-driving-trash-can
python3 scripts/build_mobile_ui.py
# Direct GPIO build:
~/.local/bin/arduino-cli compile --fqbn esp32:esp32:esp32 --jobs 2 --warnings all \
  --build-property compiler.cpp.extra_flags=-DKENNY_WIRING=1 firmware/kenny_mobile
# Carrier PCB build:
~/.local/bin/arduino-cli compile --fqbn esp32:esp32:esp32 --jobs 2 --warnings all \
  --build-property compiler.cpp.extra_flags=-DKENNY_WIRING=2 firmware/kenny_mobile
```

For upload, set `config.h` to the actual mode and compile without an override.
Identify the board with `arduino-cli board list`; never select the LiDAR adapter:

```bash
~/.local/bin/arduino-cli compile --fqbn esp32:esp32:esp32 --jobs 2 firmware/kenny_mobile
~/.local/bin/arduino-cli upload --fqbn esp32:esp32:esp32 \
  --port /dev/serial/by-id/YOUR_VERIFIED_ESP32_DEVICE firmware/kenny_mobile
```

## Phone connection and controls

| Setting | Default |
| --- | --- |
| Wi-Fi network | **KENNY-Robot** |
| Password | **KennyDrive!2026** |
| Page | **http://192.168.4.1/** |

Join the network, choose **stay connected** if the phone reports no internet,
and open the HTTP address in Safari/Chrome. Disable automatic Wi-Fi switching
if it prevents reaching the page. Verify the displayed wiring/E-stop state.

With all wheels raised for the first test: **Arm**, then hold and drag:

- Up=forward, down=reverse, left/right=turn; horizontal movement rotates in place.
- Release centers the joystick and sends exact zero requests. The session stays
  ready for another touch while fresh neutral heartbeats continue.
- **STOP & DISARM** ends the control session.
- Speed limit defaults to **50%**, scaling both speed and turn rate.
- Leaving/hiding the page, interrupted touch capture, failed/rejected commands
  disarm the UI. Re-arm deliberately to continue.
- Only one active control lease is accepted. Another browser cannot take over
  an armed controller; STOP is available to anyone on the local access point.

The default password is published in source. Change it in `config.h` before
regular use. This is a local WPA-protected AP with plain HTTP, not an
internet-facing or certified robot-control service.

## Phone-camera ArUco scanner

The **Scan ArUco** panel reads numeric marker IDs and displays each marker's
center and four corners in **source-image pixels** (origin top-left; X right,
Y down). The red corner identifies the first, orientation-normalized corner.
It supports OpenCV `DICT_4X4`, `DICT_5X5`, and `DICT_6X6` dictionaries with
50, 100, 250 or 1000 IDs. Select the dictionary used to print your markers;
the default is `DICT_4X4_50`. Detection uses exact code matching.

On **http://192.168.4.1**, use **Take a photo / choose image**. The phone can
offer its rear camera or photo picker; the resulting photo is scanned locally.
Live video requires a **secure HTTPS origin** (or localhost for development),
camera permission, and browser camera support. The ESP32 currently serves HTTP,
so its normal address uses photo scanning. No internet or CDN is needed for
detection: js-aruco2 and its dictionary data are embedded in the firmware page.

Starting scanning disarms driving; Arm is disabled while live scanning or photo
processing is active. Stop the camera before driving again. Hiding/closing the
page releases camera tracks and clears detections. Images are reduced to a
640-pixel longest edge for detection, but reported coordinates are scaled back
to the original image resolution. Frame-by-frame results are cleared when no
marker is visible.

An ArUco marker encodes an **ID**, not surveyed coordinates. This panel reports
phone-image coordinates; robot/map coordinates in meters require a marker
location database and calibrated pose estimation, plus the phone-to-robot
relationship if using the phone as a robot localization sensor.

The vendored detector is pinned to js-aruco2 revision
`0491d5d228746411d0e9dd602b98f48636a644ba`; notices are in
`firmware/kenny_mobile/web/vendor/LICENSE.txt` and the dictionary sources.
To refresh those pinned sources, run `python3 scripts/vendor_mobile_aruco.py`.
Normal UI builds use the included sources offline.

## Control limits and behavior

- Requested speed **±0.10 m/s**, turn **±0.60 rad/s**; individual wheel speeds
  rescaled together to **±0.15 m/s** while preserving curvature.
- PWM: **20 kHz**, 8-bit, commissioning cap **90/255**.
- Four encoder-feedback speed loops at approximately **100 Hz**.
- Browser commands about every **60 ms**, normally one outstanding drive request.
  Release sends a priority zero even if an earlier request is still pending.
- **300 ms** without a fresh accepted command → zero targets and disarm.
- Replays/out-of-order sequences do not refresh the watchdog. Each deliberate
  ARM creates a random lease; old packets cannot re-arm/drive a new session.
- HTTP handlers enqueue commands; a separate FreeRTOS motor task owns outputs.
  A slow HTTP client does not directly block watchdog/PID processing. Urgent
  STOP clears queued ARM/drive messages; a motor-loop gap over 100 ms disarms.

Default **1320 x4 counts/revolution** comes from 11 PPR × 30:1 × 4. Confirm it by
hand rotation. Forward rotation must increase the count; correct each motor's
`encoderSign` if necessary. The page shows all four counts, RPM and actual PWM.

KP=1.5, KI=0.5, nominal 333 RPM feedforward and 40 RPM/s target ramp are **untuned
starting values**. Direction reversal inserts at least 50 ms with enable LOW
and waits for opposing measured rotation to slow. Missing/wrong-direction
encoder feedback can latch an encoder fault; it is not current protection.

Zero PWM can **coast**, not actively brake. Verify wiring, polarity, encoder
signs, watchdog/E-stop and stopping latency on raised wheels before floor tests.
These 333 RPM motors specify **2.4 A stall current per motor**, exceeding the
common L298N continuous-channel rating; resolve driver/current/thermal sizing
before loaded use. The joystick does not avoid obstacles or detect cliffs.

## Faults and API

| Bit | Fault |
| --- | --- |
| 1 | Command watchdog expired |
| 2 | E-stop contact open |
| 4 | Invalid/nonfinite/out-of-bounds command |
| 8 | Encoder missing/opposite direction, or motor cannot move at the PWM cap |
| 16 | Control-loop timing stall |
| 32 | Encoder configuration missing |
| 64 | PWM initialization failure |
| 128 | MCP23017 I²C/latch failure; fix connection and restart |

If Arm is disabled, read mode, E-stop, ready state and faults. If another session
is still armed after a reload, use STOP or wait for watchdog expiry. Inspect
driver power, jumpers, grounds and motor/encoder signs if movement fails.

- `GET /api/status`: mode, armed/ready/E-stop/faults, counts/RPM/PWM and limits.
- `POST /api/arm`: activation; returns a new `lease`.
- `POST /api/drive`: form fields `lease`, uint32 `seq`, `v` m/s, `w` rad/s.
- `POST /api/stop`: urgent stop/disarm, no lease required.

## Verification and editing

Both modes compile with Espressif core 3.3.12. Host checks cover ownership,
numeric bounds/NaNs, stale queue, replay, counter wrap, watchdog/release/E-stop
and 10,000 wheel-limit cases. Chromium exercises the actual page at phone size:
touch drag, forward/reverse/turn, release during a pending request, STOP, blur,
command rejection and E-stop lockout. **Browser telemetry is simulated**; radio,
physical timing and motor behavior remain untested until an ESP32 is connected.

```bash
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  tests/mobile_control_test.cpp -o /tmp/kenny-mobile-control-test
/tmp/kenny-mobile-control-test
# Requires Playwright and its Chromium browser in a test venv:
python tests/check_mobile_browser.py
python tests/check_mobile_aruco.py
```

Edit `firmware/kenny_mobile/web/index.html`, then run
`python3 scripts/build_mobile_ui.py` before compiling. The complete page is
embedded in flash; no CDN, online fonts or external assets are fetched.
`artifacts/mobile/joystick-preview.png` shows **mock telemetry** from the browser
check. The Desktop package contains source/instructions, not an already-flashed
ESP32.
