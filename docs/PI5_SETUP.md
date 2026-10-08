# Kenny Raspberry Pi 5 setup

This is the reproducible installation guide for the **working sensor GUI**:
RGB image, depth image, 3D depth points and distance-colored rays.
Use Ubuntu **24.04 LTS ARM64**, Python **3.12**, and **ROS 2 Jazzy** on a Pi 5.
The tested Pi has 8 GB RAM and an original **Orbbec Astra Pro**, plus a LiDAR
on a CP2102 USB adapter. See [measured setup status](PI5_SETUP_STATUS.md) for
test results and unresolved hardware issues, and the
[robot integration guide](RL_TRAINING_AND_ROBOT_GUIDE.md) for steering/navigation.

## 1. Get the repository and dependencies

```bash
sudo apt update
sudo apt install -y git curl ca-certificates software-properties-common \
  build-essential cmake python3-venv usbutils python3-serial wmctrl
git clone https://github.com/EruzaDev/Kenny---Automotive-self-driving-trash-can.git
cd Kenny---Automotive-self-driving-trash-can
export KENNY_REPO="$PWD"
```

For an existing clone, use `git pull --ff-only` after saving local changes.

Configure the **official ROS apt repository** using the
[Jazzy Ubuntu deb instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The current `ros2-apt-source` method is:

```bash
sudo add-apt-repository universe
ROS_APT_SOURCE_VERSION=$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-rviz2 \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-ros \
  ros-jazzy-camera-info-manager ros-jazzy-image-geometry ros-jazzy-image-publisher \
  python3-numpy libuvc-dev libusb-1.0-0-dev libgoogle-glog-dev libgflags-dev \
  libeigen3-dev v4l-utils
source /opt/ros/jazzy/setup.bash
# Run init only if /etc/ros/rosdep/sources.list.d/20-default.list does not exist.
sudo rosdep init
rosdep update --rosdistro jazzy
```

## 2. Build the pinned YDLIDAR SDK and ROS driver

The following paths are new workspaces. Reuse an existing checkout only after
checking its revision/local changes; do not overwrite it.

```bash
mkdir -p ~/vendor_src ~/kenny_ws/src
git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/vendor_src/YDLidar-SDK
git -C ~/vendor_src/YDLidar-SDK checkout 42a82ed10d2304094c111fc63dee8e4a229b79b7
cmake -S ~/vendor_src/YDLidar-SDK -B ~/vendor_src/YDLidar-SDK/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build ~/vendor_src/YDLidar-SDK/build -j2
sudo cmake --install ~/vendor_src/YDLidar-SDK/build
sudo ldconfig

git clone --branch humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git \
  ~/kenny_ws/src/ydlidar_ros2_driver
git -C ~/kenny_ws/src/ydlidar_ros2_driver checkout 4ef70d3f32a85704ade0be54b214f3763b1ab3e8
cd ~/kenny_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 3. Build the Astra Pro driver with Jazzy compatibility edits

The legacy driver includes an ARM64 OpenNI runtime. The checked-in patcher
reproduces the tested header/API/CMake fixes, validates the pinned revision,
and can be rerun on an already-patched tree.

```bash
mkdir -p ~/astra_ws/src
git clone https://github.com/orbbec/ros2_astra_camera.git \
  ~/astra_ws/src/ros2_astra_camera
git -C ~/astra_ws/src/ros2_astra_camera checkout f7e71d9ce806e788cb48d8580aac2c778fba4214
python3 "$KENNY_REPO/scripts/pi5_patch_astra.py" --check
python3 "$KENNY_REPO/scripts/pi5_patch_astra.py"
cd ~/astra_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

The tested launch filename is `astra_pro.launch.xml`, not `astrapro.launch.xml`.
These edits are for Jazzy at the pinned revision, not a general port to every
ROS distribution. They do not repair the vendor's RGB/depth registration path.

## 4. USB permissions, serial port and desktop shortcut

```bash
sudo usermod -aG dialout,video "$USER"
sudo install -m 0644 "$KENNY_REPO/configs/pi5-udev.rules" \
  /etc/udev/rules.d/70-kenny-sensors.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
lsusb
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
python3 "$KENNY_REPO/scripts/pi5_install_shortcuts.py"
```

**Log out and back in** so the desktop session gets the new serial/video groups.
Set `port` in `configs/pi5-lidar.yaml` to the actual LiDAR by-id path. The
checked-in path is the tested Pi's adapter and can differ on another machine.
Confirm the LiDAR model/revision and vendor parameter table before using it.
The starting X3/X3 Pro settings are 115200 baud, single channel, 0.10–8 m,
and a requested 6 Hz rotation setting. The tested hardware's observed rate
and intermittent link still require investigation.

The helper installs `~/kenny_env.bash`, adds it to `.bashrc` if needed, and
creates **Kenny Live Sensors** on the desktop. If GNOME asks, select
**Allow Launching** for the shortcut.

## 5. Open the live GUI

Double-click **Kenny Live Sensors**, or run:

```bash
python3 "$KENNY_REPO/scripts/pi5_open_gui.py"
```

For foreground launch and console logs:

```bash
source "$KENNY_REPO/configs/pi5-env.bash"
ros2 launch "$KENNY_REPO/scripts/pi5_gui.launch.py"
```

You should see:

- Live RGB and grayscale depth image panels.
- A 3D depth cloud with lines from the camera to measured surfaces.
- **Red ≤0.5 m**, **yellow intermediate**, **green ≥3 m**. These encode range,
  not the object's RGB texture.

Drag to orbit, scroll to zoom, Shift+drag to pan. Toggle the point cloud/rays
in Displays. Closing RViz also stops the GUI's sensor and visualization nodes.
Reopening an already-running GUI brings its window forward.

To change range thresholds, edit the `pi5_depth_view.py` command in
`scripts/pi5_gui.launch.py`, for example append `--near`, `0.3`, `--far`, `2.0`.
The visualization omits zero/invalid depth and uses published camera intrinsics.

To view LiDAR separately, disable the 3D depth/ray layers, set Global Options →
Fixed Frame to `laser_frame`, and enable Live LiDAR. Calibrated camera-to-LiDAR
transforms are not supplied, so both clouds are not merged in a common frame.

## 6. Verify the setup

With the GUI running:

```bash
source "$KENNY_REPO/configs/pi5-env.bash"
ros2 node list
ros2 topic list -t
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" \
  --attach --visualization --seconds 30
```

With the GUI closed, test the native driver streams:

```bash
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" --seconds 60
# Longer stream check specified by the robot guide:
python3 "$KENNY_REPO/scripts/pi5_sensor_check.py" --seconds 1800
```

Diagnostics go to ignored `artifacts/pi5/`: `gui.log`, `live-check.json`,
`sensor-check.json`, and `sensor-bringup.log`. A nonzero checker exit means
at least one required stream is absent/stale. Inspect individual topic counts;
the depth GUI can work while LiDAR or the native vendor point cloud fails.
Stream reception alone does not validate geometry or stopping behavior.

To verify the visualization math without hardware (units, row padding,
endianness, invalid depth, range colors and camera-frame matching):

```bash
source "$KENNY_REPO/configs/pi5-env.bash"
python3 -m pytest -q "$KENNY_REPO/tests/test_pi5_depth_view.py"
```

These five tests passed with system ROS Python. They are skipped when running
the simulator's isolated venv without ROS; that suite passed with 82 tests and
two skips after adding the visualization checks.

### Troubleshooting

| Symptom | Check |
| --- | --- |
| Cannot open LiDAR serial port | Adapter present in `lsusb`/by-id, correct port, `dialout` group, no competing driver |
| LiDAR timeouts / no scan | USB data cable, motor power/PWM, spinning head, physical revision/baud rate |
| No camera images | Both Astra USB IDs (`2bc5:0403`, `2bc5:0501`), `video` group, udev rules, `gui.log` |
| Empty 3D view | Valid depth, nonzero depth intrinsics, `/kenny/depth/points`, Fixed Frame `camera_link` |
| Lag | Disable unnecessary displays; reduce visualization `--fps`; profile before lowering sensor coverage |
| Blank RGB-textured vendor cloud | Registration path is unvalidated; use the working distance-colored depth view |

## 7. Optional simulator and model check

This is separate from system ROS Python. On ARM64 the pinned PyTorch wheel
is available from PyPI; the laptop's CPU-wheel index is not needed here.

```bash
cd "$KENNY_REPO"
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install --upgrade pip
env -u PYTHONPATH .venv/bin/python -m pip install -e '.[train,view,test]' \
  torch==2.5.1 pyserial esptool
env -u PYTHONPATH .venv/bin/python -m pip check
env -u PYTHONPATH .venv/bin/python -m pytest -q
env -u PYTHONPATH .venv/bin/python -m kenny_rl.demo --stage full \
  --snapshot artifacts/pi5/world.png
```

The included trained checkpoint works in simulation. No ESP32 was present
during this setup. Motor firmware, Pi motor bridge, calibrated robot frames,
localization, perception/stop chain and real policy feature building remain
the next integration steps in the robot guide.
