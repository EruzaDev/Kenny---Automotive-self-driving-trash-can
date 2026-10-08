# Raspberry Pi 5 installation and measured status

Installed on `kenny-desktop`, 2026-10-08, following
[the robot setup guide](RL_TRAINING_AND_ROBOT_GUIDE.md), especially sections 7–9.
For installation on a fresh Pi, use the [reproducible setup guide](PI5_SETUP.md),
including the checked-in Astra patcher and desktop/environment installer.

## Installed

- Raspberry Pi 5 Model B Rev 1.0, 8 GB RAM, Ubuntu 24.04.5 ARM64,
  Python 3.12.3, synchronized system clock.
- Official `ros2-apt-source` repository, ROS 2 Jazzy ROS Base, Nav2 and
  bringup, Collision Monitor, SLAM Toolbox, robot localization, robot state
  publisher, xacro, TF2, cv_bridge, image transport and ROS development tools.
- YDLIDAR SDK under `~/vendor_src/YDLidar-SDK`, installed in `/usr/local`.
  ROS driver built in `~/kenny_ws` from the vendor's `humble` branch.
- Original Astra Pro legacy OpenNI driver built in separate `~/astra_ws`.
  Its bundled ARM64 OpenNI runtime successfully opened the connected camera.
- Repository `.venv` with NumPy 1.26.4, Gymnasium 1.0.0, PyTorch 2.5.1
  ARM64, Stable-Baselines3 2.4.1, plotting/testing dependencies, PySerial and
  esptool 5.5.0.
- `kenny` added to `dialout` and `video`. Log out and back in to refresh
  groups in an existing desktop/terminal session.
- Sensor USB permissions in `/etc/udev/rules.d/70-kenny-sensors.rules`.
- `~/kenny_env.bash` sourced by `~/.bashrc` for new interactive Bash terminals.
  It loads ROS and both sensor workspace overlays. The RL venv is separate
  from system ROS Python.

## Start sensors

### Live desktop GUI: 3D depth and distance-colored rays

Double-click **Kenny Live Sensors** on the Pi desktop, or run:

```bash
python3 ~/Kenny---Automotive-self-driving-trash-can/scripts/pi5_open_gui.py
```

RViz opens with live RGB and grayscale depth image panels, a 3D measured-depth
point cloud, and up to 384 rays extending from the camera to sampled surfaces.
**Red means range ≤0.5 m, yellow is intermediate, green means range ≥3 m.**
These colors encode camera-to-surface distance, not the object's RGB color.
The projection uses the depth camera's published intrinsics; zero/invalid depth
is omitted rather than drawn as free space. The view uses a subsampled cloud
and targets 5 updates/s to limit Pi load. Thresholds can be adjusted in
`scripts/pi5_depth_view.py`'s launch command using `--near` and `--far`.

- Drag with the left mouse button to orbit, scroll to zoom, and use Shift+drag
  to pan in RViz's Orbit view.
- Toggle **Depth Rays** or **3D Depth** in Displays to inspect either layer.
- Closing RViz stops the GUI's drivers and visualization node.
- For LiDAR, disable the depth 3D/ray layers, change Global Options → Fixed
  Frame to `laser_frame`, and enable **Live LiDAR**. There is no measured
  LiDAR-to-camera transform yet, so the two 3D frames are not merged.
- GUI logs: `artifacts/pi5/gui.log`. Verified screenshot:
  `artifacts/pi5/depth-rays-gui.png`.

The GUI publishes `/kenny/depth/points` (`PointCloud2` with x/y/z/rgb fields)
and `/kenny/depth/rays` (`MarkerArray`). In a 30-second check with RViz active,
the view produced approximately 3.5 updates/s, 13,074 valid cloud points and
245 valid rays in the last observed frame. A sample contained 204 distinct
distance colors. The rendered window showed both camera images, the 3D cloud
and the red/yellow/green rays, with RViz Global Status OK.

### Recheck findings (2026-10-08)

The desktop GUI and camera/depth-ray visualization work. The Pi reported no
failed system services, 127 GB free disk space, and approximately 59.5°C before
starting the GUI. **The complete hardware check does not currently pass:**
LiDAR worked on one retry but returned no scans on subsequent checks; its USB
adapter disappeared/reappeared and driver startup logged serial-open/timeouts.
Check LiDAR USB/data and power connections and whether the head is spinning.
Its physical revision, rotation/PWM and scale still require confirmation.

The vendor's RGB-textured registered cloud was tried, but no registered cloud
messages were received and registered depth camera-info intrinsics were zero.
The working distance-colored GUI therefore uses the original, unregistered
depth stream with valid intrinsics. Actual RGB-textured 3D fusion needs the
registration/calibration path repaired; it is not claimed by this GUI.

To inspect the running GUI without starting duplicate sensor drivers:

```bash
source ~/kenny_env.bash
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" --attach --visualization --seconds 30
```

Results are written to `artifacts/pi5/live-check.json`. This check reports all
sensor streams, so it exits with failure when LiDAR or the vendor-native point
cloud is absent even if the custom depth visualization is receiving data.

In a new terminal, or after logging back in:

```bash
source ~/kenny_env.bash
ros2 launch "$KENNY_REPO/scripts/pi5_sensors.launch.py"
```

This launch starts sensors only. Ctrl+C stops both drivers.
LiDAR parameters are in `configs/pi5-lidar.yaml`; the serial device uses
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
The launch uses the actual vendor filename `astra_pro.launch.xml` and disables
the unused IR image stream. Depth registration remains off: RGB and depth
pixels must not be assumed to correspond.

To repeat the measured check with the normal sensor launch stopped:

```bash
source ~/kenny_env.bash
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" --seconds 60
# The guide's longer soak-test duration:
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" --seconds 1800
```

The checker starts/stops both drivers and writes
`artifacts/pi5/sensor-check.json` and `sensor-bringup.log` (overwritten on each
check). A successful exit means all listed streams were received; it does
not validate calibration, stream stability, stopping latency or navigation.

## Verified results

- Existing functional suite: **82 passed, 1 skipped**.
- Full simulator benchmark: 1,000 steps, **32.34 steps/s**. Peak RSS reported
  by the benchmark was approximately **1,033 MiB** during package installation.
- All bundled `releases/dynamic-v3-seed2/SHA256SUMS` entries verified.
- Bundled `model/best.zip` loads and predicts finite actions on ARM64.
  On one simulated 1,076-element observation, 1,000 deterministic CPU
  predictions with two PyTorch threads measured p50 **2.43 ms**, p95
  **13.90 ms**, maximum **20.97 ms**, while the sensor check was running.
  This is inference-only timing, not the sensor-to-motor latency.
- Simultaneous **60-second** ROS subscriber check:

| Topic | Received messages | Observed subscriber rate |
| --- | ---: | ---: |
| `/scan` | 688 | 12.18 Hz |
| `/camera/color/image_raw` | 546 | 9.78 Hz |
| `/camera/color/camera_info` | 1,518 | 26.81 Hz |
| `/camera/depth/image_raw` | 933 | 16.49 Hz |
| `/camera/depth/camera_info` | 1,598 | 28.12 Hz |
| `/camera/depth/points` | 43 | 0.76 Hz |

RGB/depth were 640×480, encoded `rgb8`/`16UC1`, with nonzero intrinsics.
Camera serial number: `17121410562`. USB IDs: RGB `2bc5:0501`,
depth `2bc5:0403`. The check receives and deserializes all streams in one
Python process, so these rates are not isolated driver throughput benchmarks.
Image and especially point-cloud throughput need profiling before navigation.

LiDAR returns scans on the CP2102 adapter. Initial parameters come from the
vendor SDK's X3/X3 Pro table (115200 baud, single channel, 0.10–8 m, nominal
3 kHz sampling, 4–8 Hz PWM rotation). The observed scan rate is higher than
that table and startup logs contain checksum/baseplate-identification errors.
Confirm the physical model/revision, adapter power/PWM and measured distances
before treating these settings as calibrated. Variable scan resolution avoids
the vendor's repeated fixed-point truncation warnings.

The 30-minute soak test, timestamp drift, thermal steady state, measured range
scale/orientation, RGB/depth registration and actual marker detection have
**not** been validated by the short check.

## Use the simulator / installed model

```bash
cd ~/Kenny---Automotive-self-driving-trash-can
source .venv/bin/activate
python -m kenny_rl.demo --stage full --snapshot artifacts/pi5/world.png
python -m kenny_rl.demo --model releases/dynamic-v3-seed2/model/best.zip --stage dynamic
```

The demo and model operate in the simulator. Hardware feature construction,
localization, robot description/calibrated transforms, mission control, the
motor interface and complete stop chain remain implementation work described
in the robot guide. Installing Nav2 does not supply those custom Kenny nodes.

## Add the ESP32 later

The owner confirmed **no ESP32 is connected yet**. The existing CP2102 port
is serving LiDAR and must not be selected as the ESP32 motor port.

When the ESP32 is connected over a USB data cable:

1. Identify its new port with `python3 -m serial.tools.list_ports -v` or
   `.venv/bin/python -m serial.tools.list_ports -v`, and use its stable by-id
   path. If its adapter has a duplicate serial number, match the board by
   physical USB path rather than reusing the LiDAR identity.
2. Record exact ESP32 board, motor driver, motor/encoder pin wiring, encoder
   levels/counts, direction polarity and power arrangement.
3. Implement or supply firmware with wheel-speed PID, encoder telemetry and
   faults, explicit command units/protocol and a starting 200 ms watchdog.
   The Pi bridge must match that firmware; there is no existing Kenny MCU
   protocol in this repository to configure or flash.
4. Feed only fresh bounded commands from `/cmd_vel/safe` through the future
   motor bridge. Convert requested forward/turn velocity to wheel targets
   using measured wheel radius and track width (provisional values: 0.04 m
   radius, 0.235 m track). Begin hardware trials at the guide's 0.10 m/s.
5. Verify motor/encoder signs and stopping on serial loss or a crashed Pi
   process with wheels raised before floor trials. Finish the guide's
   calibration, perception and cliff-interlock integration before autonomous
   operation.

## Driver revisions and local compatibility patches

| Source | Revision |
| --- | --- |
| Kenny | `297925dad5441ef19d38e540e6d890a816dbc890` |
| YDLidar-SDK | `42a82ed10d2304094c111fc63dee8e4a229b79b7` |
| ydlidar_ros2_driver (`humble`) | `4ef70d3f32a85704ade0be54b214f3763b1ab3e8` |
| ros2_astra_camera | `f7e71d9ce806e788cb48d8580aac2c778fba4214` |

The Astra checkout contains local Jazzy compatibility changes: `.hpp` headers
for image_geometry/cv_bridge, updated `OnSetParametersCallbackType`, explicit
`<cstdint>` inclusion and preservation of the requested Release build type.
They are uncommitted and inspectable with
`git -C ~/astra_ws/src/ros2_astra_camera diff`.
`scripts/pi5_patch_astra.py` reproduces those edits; its output was verified
to exactly match the tested driver's diff on a clean pinned worktree.
Rebuild with:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/astra_ws
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```
