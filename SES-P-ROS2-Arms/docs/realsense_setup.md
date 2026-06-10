# Intel RealSense setup

`machine_vision_pkg` consumes synchronised RGB + aligned-depth from an Intel
RealSense camera (tested on a D455). This is the only external dependency
that **cannot** be installed via `pip install -r requirements.txt` — it ships
as a ROS 2 apt package and pulls in Intel's `librealsense2` SDK as a system
dependency. The steps below are the minimum needed to bring the camera up on
Ubuntu 22.04 + ROS 2 Humble.

## 1. Install the ROS 2 wrapper

```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera
```

This single command also pulls in `librealsense2`, `librealsense2-utils`, and
the udev rules needed for non-root access to the camera. No manual repo setup
is required on stock ROS 2 Humble.

## 2. Plug the camera into a USB 3 port

The D455 needs USB 3.0 (or better) to publish 720p / 30 fps streams. A USB 2
port will negotiate but throttle resolution and frame rate. The blue USB
ports on most laptops are USB 3.

After plugging in, verify the camera is detected:

```bash
rs-enumerate-devices -s
```

You should see one line of output naming the device (`Intel RealSense
D455 ...`).

## 3. Smoke-test the ROS 2 wrapper

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=true \
    align_depth.enable:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```

Then in another terminal, confirm the topics are publishing:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
```

All three should show traffic at ~30 Hz (the `camera_info` topic once). Once
that works, `machine_vision_pkg`'s launch files will pick the streams up
automatically.

## Troubleshooting

- **`No device detected`** — make sure the cable supports data (some USB-C
  cables are power-only) and the port is USB 3. Try a different cable or
  port before suspecting the camera.
- **`Failed to read busnum/devnum` HID warnings during launch** — harmless.
  These come from librealsense's IMU back-end and do not affect colour or
  depth streams.
- **`Asic Temperature value is not valid!`** — also harmless. An intermittent
  reading from the depth ASIC's temperature sensor that librealsense ignores.
- **Frames not synchronised, or large depth-to-colour drift** — make sure
  `enable_sync:=true` and `align_depth.enable:=true` are both set; the
  `objects_in_3D.launch.xml` file in this package already does this.
- **Firmware update** — if `rs-enumerate-devices -s` shows a firmware
  significantly older than `5.17.0.10`, update it via
  `intel-realsense-dfu` from the librealsense tools.
