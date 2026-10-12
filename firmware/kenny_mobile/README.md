# KENNY mobile joystick

Open **kenny_mobile.ino** with **config.h**, **control.h** and **web_ui.h** beside
it. Arduino IDE board: **ESP32 Dev Module**, Espressif core **3.3.12**.

Select the actual wiring in config.h before upload:

- **KENNY_DIRECT_GPIO** (default): earlier independent L298N GPIO connections;
  E-stop GPIO15.
- **KENNY_CARRIER_MCP23017**: new carrier PCB; expander directions, E-stop GPIO18.

Phone Wi-Fi: **KENNY-Robot**, password **KennyDrive!2026**, page
**http://192.168.4.1/**. Arm, hold/drag, release to zero; STOP disarms. Motors
start disabled. No internet/app/additional Arduino libraries are needed.

The **Scan ArUco** panel uses the phone camera/photo picker to read marker IDs
and image center/corner coordinates. Select your OpenCV marker dictionary.
Use **Take a photo / choose image** at the ESP32 HTTP address; live video needs
HTTPS. Camera scanning disarms driving. Coordinates are pixels, not map meters.

Full guide: [`docs/ESP32_MOBILE_CONTROLLER.md`](../../docs/ESP32_MOBILE_CONTROLLER.md).
Edit `web/index.html`, then regenerate web_ui.h with `scripts/build_mobile_ui.py`.
Test on raised wheels and resolve driver sizing for the 2.4 A-stall motors
before loaded operation. Software tests do not validate physical hardware.
