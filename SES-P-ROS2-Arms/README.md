# `SES-P-ROS2-Arms/` -- Reference ROS 2 workspace

This folder is a complete colcon workspace for the SES-Pro 550 mm 6-DOF arm,
shipped at the state I actually use on my bench. The intent is to give a
working snapshot you can build and run directly, without having to apply
every patch from the top-level [README](../README.md) by hand.

## Layout

```
SES-P-ROS2-Arms/
└── src/
    ├── machine_vision_pkg/      ← original work for this project (documented below)
    ├── pro_arm_description/     ← upstream Lynxmotion, with patches applied
    ├── pro_arm_moveit/          ← upstream Lynxmotion, with patches applied
    ├── pro_hardware_interface/  ← upstream Lynxmotion, with patches applied
    ├── pro_sim_examples/        ← upstream Lynxmotion
    ├── images/                  ← upstream Lynxmotion docs media
    ├── README.md                ← upstream README
    └── required.repos
```

All four `pro_*` packages (and the supporting `images/`, `required.repos`,
upstream `src/README.md`) come from the official Lynxmotion
[SES-P-ROS2-Arms](https://github.com/Lynxmotion/SES-P-ROS2-Arms) repository.
The edits documented in the [ROS2 Control](#ros2-control) section below have
already been applied in place — so this workspace builds and runs against
the real / fake / sim arm out of the box.

`machine_vision_pkg/` is the only package that does **not** come from
upstream; everything after the patches section in this README is about that
one.

## ROS2 Control

This section is a record of every patch that has been applied to the upstream
`pro_*` packages bundled here, alongside step-by-step instructions for anyone
who wants to reproduce them on a fresh clone of
[`Lynxmotion/SES-P-ROS2-Arms`](https://github.com/Lynxmotion/SES-P-ROS2-Arms).
File paths in the instructions below assume the upstream-style workspace
location (`~/SES-P-ROS2-Arms/src/...`); when working inside *this* repo, read
those paths as relative to this workspace's `src/`.

1. After installing the ROS2 packages for control by following the [official docs](https://github.com/Lynxmotion/SES-P-ROS2-Arms), just do the edit below to avoid any control plugin errors. Edit the line 61 in at `~/SES-P-ROS2-Arms/src/pro_arm_decription/launch/view_ign.launch.py` as below:
   - Original:
     ```python
     finger
     ```
   - Edited:
     ```python
     finger,
     " ",
     "ros2_control_plugin:=sim",
     ```
   Then, rebuild and source the workspace.

2. The `~/SES-P-ROS2-Arms/src/pro_arm_moveit/launch/move_arm.launch.py` (which is included by `sim_arm_control.launch.py`) still unconditionally starts a standalone `ros2_control_node`. In `sim` mode this node crashes because `gz_ros2_control` (running inside the Gazebo process) already hosts its own `controller_manager` and owns the `ign_ros2_control/IgnitionSystem` hardware interface; the standalone node can't load that class because it filters by base type `hardware_interface::SystemInterface`. The crash looks like:
     ```
     [ros2_control_node-3] terminate called after throwing an instance of 'pluginlib::LibraryLoadException'
     [ros2_control_node-3]   what():  ... the class ign_ros2_control/IgnitionSystem with base class type hardware_interface::SystemInterface does not exist.
     [ERROR] [ros2_control_node-3]: process has died [pid ..., exit code -6, ...]
     ```

     The simulation still works (`gz_ros2_control` loads the controllers itself), but the noisy crash should be suppressed. Edit `~/SES-P-ROS2-Arms/src/pro_arm_moveit/launch/move_arm.launch.py` in two places.

     **Edit A -- line 11, add `UnlessCondition` to the imports:**
     - Original:
       ```python
       from launch.conditions import IfCondition
       ```
     - Edited:
       ```python
       from launch.conditions import IfCondition, UnlessCondition
       ```

     **Edit B -- around line 234, add a `condition=` to the `ros2_control_node` Node so it only starts when the plugin is *not* `sim`:**
     - Original:
       ```python
       Node(
           package="controller_manager",
           executable="ros2_control_node",
           output="log",
           arguments=["--ros-args", "--log-level", log_level],
           parameters=[
               robot_description,
               controller_parameters,
               {"use_sim_time": use_sim_time},
           ],
       ),
       ```
     - Edited:
       ```python
       Node(
           package="controller_manager",
           executable="ros2_control_node",
           output="log",
           arguments=["--ros-args", "--log-level", log_level],
           parameters=[
               robot_description,
               controller_parameters,
               {"use_sim_time": use_sim_time},
           ],  
           # In sim mode, gz_ros2_control hosts controller_manager inside Gazebo,
           # so the standalone ros2_control_node would fail to load IgnitionSystem.
           condition=UnlessCondition(
               PythonExpression(["'", ros2_control_plugin, "' == 'sim'"])
           ),  
       ),  
       ``` 
       
     Then rebuild and source the workspace. This change is safe for all three modes:
     - `fake` (default for `move_arm.launch.py`, `fake_arm_control.launch.py`) → standalone
  `ros2_control_node` still starts and loads `fake_components/GenericSystem`.
     - `sim` (`sim_arm_control.launch.py`) → standalone node is skipped; `gz_ros2_control`'s in-process
  `controller_manager` handles everything.
     - `real` (`real_arm_control.launch.py`) → standalone node still starts and loads 
  `pro_motor_hardware/ProMotorHardware` to talk to the servos.

3. Optional cosmetic cleanup: silence the `ign_ros2_control` plugin got renamed to `gz_ros2_control`
  deprecation warning printed by Gazebo during `sim` mode. Both class names refer to the same C++ class
  on ROS 2 Humble -- `ign_ros2_control/IgnitionSystem` is the old name kept for backward compatibility,
  `gz_ros2_control/GazeboSimSystem` is the current one. Switching is purely a name change and does not
  affect simulation behavior. The warning looks like:
     ```
     [ign gazebo-1] [WARN] [gz_ros2_control]: The ign_ros2_control plugin got renamed to gz_ros2_control.
     Update the <ros2_control> tag and gazebo plugin to
     <hardware>
       <plugin>gz_ros2_control/GazeboSimSystem</plugin>
     </hardware>
     ```
     Edit `~/SES-P-ROS2-Arms/src/pro_arm_description/urdf/pro_arm.ros2_control` and replace **both**
  occurrences (lines 16 and 172 — one for the arm `ros2_control` block, one for the gripper block).
     - Original:
       ```xml
       <plugin>ign_ros2_control/IgnitionSystem</plugin>
       ```
     - Edited:
       ```xml
       <plugin>gz_ros2_control/GazeboSimSystem</plugin>
       ```
     Then rebuild and source the workspace.

4. Set explicit per-joint `max_velocity` values in `~/SES-P-ROS2-Arms/src/pro_arm_moveit/config/joint_limits.yaml`*so MoveIt's Time-Optimal Trajectory Generation (TOTG) stops falling back to its 1 rad/s default. The shipped file declares `has_velocity_limits: false` for every joint, which triggers this warning at launch:
   ```
   [moveit_trajectory_processing.time_optimal_trajectory_generation]: Joint velocity
   limits are not defined. Using the default 1 rad/s. You can define velocity limits
   in the URDF or joint_limits.yaml.
   ```
   So set `has_velocity_limits: true` and an explicit `max_velocity` (must be rad/s; deg/s shown alongside for clarity) per arm joint in `~/SES-P-ROS2-Arms/src/pro_arm_moveit/config/joint_limits.yaml`, matching the URDF ceilings:
   - J1, J2, J3 (S1 / standard servo): `1.57 rad/s` ≡ `90 °/s`.
   - J4, J5 (L1 / lite servo): `1.31 rad/s` ≡ `75 °/s`.
   - J6: capped at `1.31 rad/s` ≡ `75 °/s` for consistency with J4/J5 (URDF allows up to `6.283 rad/s` ≡ `360 °/s`).

   These are MoveIt planning ceilings only — the real arm physically tops out at ≈ `0.087 rad/s` ≡ `5 °/s` on every joint.

   Example block for `pro_arm_joint_1` (apply analogous edits to all six arm joints, using `1.31` for joints 4–6):
   ```yaml
   pro_arm_joint_1:
     has_velocity_limits: true
     max_velocity: 1.57     # rad/s (ceil: 90 deg/s, actual movement: 5 deg/s)
     has_acceleration_limits: true
     max_acceleration: 1.74  # rad/s^2 (~99.7 deg/s^2)
   ```

   Then rebuild and source the workspace.

5. **USB port permissions for the real arm.** The `pro_motor_hardware` driver needs read/write access to `/dev/ttyACM0`. By default the device is owned by `root:dialout` with mode `crw-rw----`, which excludes regular users -- without this the driver fails immediately at `open()` with `Permission denied`. Two options:
   - **One-shot per boot** (the official docs' suggestion; needs to be re-run after every reboot or unplug/replug):
     ```bash
     sudo chmod 666 /dev/ttyACM0
     ```
   - **Persistent (recommended)** — add yourself to the `dialout` group once, then log out and back in (or reboot) for the new group to take effect:
     ```bash
     sudo usermod -aG dialout $USER
     ```

6. **Termios setup in `~/SES-P-ROS2-Arms/src/pro_hardware_interface/pro_motor_hardware/src/pro.c`** ⚠️ critical.

- Issue: `ros2 launch pro_arm_moveit real_arm_control.launch.py` silently hangs forever during hardware init -- the spawner nodes time out on `/controller_manager/list_controllers`, the controllers never come up, and joint 1's LED enters the red/white blinking pattern (LSS-P `QLI = 8` = "Unknown Serial Command").
- Root cause: `pro_init_bus()` only sets `c_cflag` (control) flags and leaves `c_iflag`/`c_oflag`/`c_lflag` at the kernel's cooked-mode defaults (`ICRNL`, `IXON`, `OPOST`, `ICANON`, `ECHO`, ...). Those defaults mangle the LSS-P framing -- on input, `\r` (the LSS-P terminator) is mapped to `\n`; on output, post-processing alters the byte stream -- so the very first bytes the driver sends to joint 1 are not parsed by the servo, and the read loop then times out waiting for a reply that never comes.
- Solution: Edit `pro.c` in two places.

     **Edit A -- around line 29, immediately after the `tcgetattr(...)` if-block: add `cfmakeraw(&tty)`.**
     - Original:
       ```c
       struct termios tty;
       if (tcgetattr(g_serial_fd, &tty) != 0) {
           printf("Failed to get port attributes (errno: %d - %s)\n", 
                  errno, strerror(errno));
           close(g_serial_fd);
           return false;
       }

       // Set baud rate
       ```
     - Edited:
       ```c
       struct termios tty;
       if (tcgetattr(g_serial_fd, &tty) != 0) {
           printf("Failed to get port attributes (errno: %d - %s)\n", 
                  errno, strerror(errno));
           close(g_serial_fd);
           return false;
       }

       // Raw-mode baseline: clears ICRNL/IXON/OPOST/ICANON/ECHO so LSS-P
       // framing (\r terminator) passes through unmangled.
       cfmakeraw(&tty);

       // Set baud rate
       ```

     **Edit B -- around line 66, immediately before the `tcsetattr(...)` call: add `tcflush(g_serial_fd, TCIOFLUSH)`.**
     - Original:
       ```c
       if (tcsetattr(g_serial_fd, TCSANOW, &tty) != 0) {
           close(g_serial_fd);
           return false;
       }
       ```
     - Edited:
       ```c
       tcflush(g_serial_fd, TCIOFLUSH);   // drop any stale RX/TX bytes
       if (tcsetattr(g_serial_fd, TCSANOW, &tty) != 0) {
           close(g_serial_fd);
           return false;
       }
       ```

     Then rebuild and source the workspace. After this fix the real-arm launch progresses past port setup; the visual-debug procedure (set `CLED 0` on every servo via PRO Config so the LEDs are dark at boot, then launch and watch) shows all six joints transitioning cleanly through `on_configure` (brief green flashes) and `on_activate` (brief blue flashes).

7. **Joint 4 `max_speed` typo in `pro_arm.ros2_control`.** After the termios fix the launch comes up cleanly, but joint 4 immediately enters a persistent error state and refuses to move -- LED blinks 4× red then pauses, status reads "Error" and the error field shows "Speed Setting Exceeds Limit" (LSS-P `QLI = 3`). Cause: the URDF tells `pro_motor_hardware` to push joint 4 to `90 °/s`, but joint 4 is an **L1 (lite) servo** with a hardware ceiling of `75 °/s`. Joints 5 and 6 (also L1) are correctly set to 75 -- only joint 4 has the typo. Edit `~/SES-P-ROS2-Arms/src/pro_arm_description/urdf/pro_arm.ros2_control` line 104:
   - Original:
     ```xml
     <joint name="${prefix}joint_4">
       <param name="id">4</param>
       <param name="max_speed">90</param>
       <param name="acceleration">50</param>
     ```
   - Edited:
     ```xml
     <joint name="${prefix}joint_4">
       <param name="id">4</param>
       <param name="max_speed">75</param>
       <param name="acceleration">50</param>
     ```

   Then rebuild and source the workspace. Before relaunching, open PRO Config with `Select ID = 4` and click **RESET** (top right) to clear joint 4's persistent error state — the servo will keep rejecting commands until its stored fault is cleared, even with the URDF fixed.

## Build

```bash
cd SES-P-ROS2-Arms
rosdep install --from-paths src --ignore-src -r -y      # one-time
colcon build --symlink-install
source install/setup.bash
```

To build only the vision package (assuming the upstream `pro_*` packages
are already built and installed):

```bash
colcon build --symlink-install --packages-select machine_vision_pkg
```

---

## `machine_vision_pkg/`

A ROS 2 Python package that turns an Intel RealSense D455 mounted above the
arm into a closed-loop vision system for the 550 mm pro_arm, using:

- a two-stage classifier -- **YOLOv8-seg** for high-level mechanical classes
  (`gear`, `nut`, `screw`) plus a **ConvNeXt-Tiny image-to-geometry** Stage-2
  for fine-grained low-level classes (`gear_a`, `nut_b`, `screw_f`, …),
- 3D reprojection from camera pixels into the arm base frame using the
  RealSense intrinsics + a user-supplied static extrinsic,
- **MoveIt 2** for collision-aware IK and OMPL trajectory planning, with a
  workshop-table obstacle pushed into the planning scene at startup so the
  arm refuses to move below the table or through it.

### Nodes

| Executable                  | What it does |
|-----------------------------|--------------|
| `realsense_rgbd_viewer`     | Lightweight subscriber that displays the synchronized colour + aligned-depth streams in two OpenCV windows. Useful for camera sanity checks. |
| `two_stage_live_classifier` | Synchronously consumes RGB + aligned-depth + camera-info, runs YOLOv8-seg, then runs Stage-2 ConvNeXt on each crop. Publishes `vision_msgs/Detection2DArray` on `/classifier/detections`. Includes a `std_srvs/SetBool` gate (`~/set_active`) so the heavy inference can be paused while the arm is moving. |
| `reprojection_3D`           | Subscribes to `/classifier/detections` + the synchronized depth/info, deprojects each bbox centre to a 3D point in the camera frame, then transforms to the arm base frame via the configured static extrinsic. Publishes `vision_msgs/Detection3DArray` on `/reprojection_3D/targets`. |
| `move_to_a_point`           | One-shot helper: moves the EE to a `(x, y, z, roll, pitch, yaw)` target with a configurable hover offset. Useful for bench testing and as the "go-to" primitive used by other nodes. |
| `pick_and_place_example`    | Hardcoded pick-and-place demo, mirroring `pro_sim_examples/ex_move_object.cpp`. Demonstrates planning-scene collision objects, `AttachedCollisionObject`, and a multi-stage MoveIt sequence on the real arm. |
| `classify_pick_and_place`   | The full vision-driven loop -- see [below](#classify_pick_and_place-loop). |

### Launch files

| File | Brings up |
|------|-----------|
| `realsense_rgbd_viewer.launch.xml`   | RealSense + viewer.                                                                  |
| `two_stage_live_classifier.launch.xml` | RealSense + classifier.                                                              |
| `objects_in_3D.launch.xml`           | RealSense + classifier + 3D reprojection. This is the "perception only" stack.       |
| `move_to_a_point.launch.py`          | Real arm (MoveIt + controllers + drivers) + `move_to_a_point` node.                  |
| `pick_and_place_example.launch.py`   | Real arm + the hardcoded pick-and-place demo.                                        |
| `classify_pick_and_place.launch.py`  | Real arm + perception stack + vision-driven loop (Python launch with TimerAction).   |
| `classify_pick_and_place.launch.xml` | Same as above, but as a declarative XML launch.                                      |

### Config

YAML files live in `config/` and supply every tunable parameter:

- `two_stage_live_classifier.yaml` -- YOLO/Stage-2 thresholds, devices,
  temporal-consistency settings (`confirm_frames`, `track_match_distance_px`),
  high-to-low class mapping.
- `reprojection_3D.yaml` -- camera intrinsics topic plumbing, depth median
  window size, and the static camera-in-arm extrinsic (position + RPY in
  degrees).

Edit those rather than overriding parameters on the CLI.

### `classify_pick_and_place` loop

Pseudocode of the main loop:

```
move_to_default()           # park at home
loop:
    enable classifier (SetBool true)        # ~/set_active service
    wait `settle_sec` for tracker to mature
    wait for one fresh /reprojection_3D/targets msg
    disable classifier (SetBool false)
    if no targets: exit
    move to first target (z += target_offset_z)
    sleep `wait_at_target_sec`
    move to user-supplied drop point
    sleep `wait_at_drop_sec`
    move_to_default()
```

Two design points worth flagging:

1. **The classifier is gated.** YOLO + ConvNeXt are the heaviest workload in
   the pipeline. The loop only enables them while the arm is parked at home
   collecting a fresh detection batch; during all motion phases the SetBool
   gate makes `process()` early-return, so the CPU/GPU is free.
2. **Collision-aware planning.** Each motion node calls
   `add_table_obstacle()` once at startup, which publishes the workshop table
   as a `CollisionObject` into MoveIt's planning scene. With
   `avoid_collisions=True` already set in the IK request, points below or
   inside the table return `NO_IK_SOLUTION`, and OMPL avoids any joint path
   that would intersect the table.

### Example: vision-driven pick-and-place

```bash
ros2 launch machine_vision_pkg classify_pick_and_place.launch.xml \
    drop_x:=0.0 drop_y:=-0.25 drop_z:=0.10
```

Set the camera mount pose in `config/reprojection_3D.yaml` and the table
geometry via the `table_*` ROS parameters declared by
`moveit_arm_base_node.py` (see the top-level README and the docstring there).

### Models

The two trained model weights are too large for git and are hosted as
release assets on this repository instead of being committed.

| File | Used by | Download |
|------|---------|----------|
| `best_2.pt` | Stage-1 YOLOv8-seg detector | [download link — TODO](https://example.com/best_2.pt) |
| `image_to_geometry_best.pth` | Stage-2 ConvNeXt-Tiny image-to-geometry classifier | [download link — TODO](https://example.com/image_to_geometry_best.pth) |

After downloading, drop both files into:

```
SES-P-ROS2-Arms/src/machine_vision_pkg/models/
```

(The directory is shipped in the repo; only the weights are external.) The
classifier loads them from that path by default; you can override either
with the `yolo_model_path` / `stage2_model_path` ROS parameters if you keep
them somewhere else.

### External dependencies (beyond standard ROS 2 Humble + MoveIt 2)

Two groups: Python libraries installed in one shot via `requirements.txt`,
and the RealSense ROS 2 wrapper installed via apt.

**Python / pip** -- see [`requirements.txt`](requirements.txt) at the root of
this workspace. Every dep is exact-pinned (`==`) to the version used on the
bench, since the YOLOv8 + ConvNeXt + trimesh stack is sensitive to
torch / numpy / scipy ABI mismatches. The file lists:

- `numpy`, `scipy`, `opencv-python`, `Pillow` -- numerical and image base.
- [`ultralytics`](https://github.com/ultralytics/ultralytics) -- YOLOv8-seg
  (Stage 1).
- `torch`, `torchvision` -- ConvNeXt-Tiny backbone (Stage 2).
- `trimesh` -- geometry-prototype generation from the bundled `.glb` assets.
- `scikit-learn` -- only needed if you re-run `train_image_to_geometry.py`;
  not required for inference.

Install everything in one shot with:

```bash
pip install -r requirements.txt
```

A clean virtualenv is strongly recommended -- pinned pip versions of
`numpy` / `scipy` / `Pillow` / `opencv-python` can collide with the
apt-installed `python3-*` versions that ROS 2 Humble ships.

**System / apt** --
[`realsense2_camera`](https://github.com/IntelRealSense/realsense-ros) is
Intel's official ROS 2 wrapper and the only dep that isn't pip-installable.
See [`docs/realsense_setup.md`](docs/realsense_setup.md) for the install,
smoke test, and common pitfalls.
