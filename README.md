# Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm

## General

1. Useful links.
   - Documentation: [Here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-arms/ses-pro-550-6-dof-arm/)
   - SES-PRO Robotic Arm UI software: [Here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-software/ses-pro-arm-ui/)

2. Joint Limites (deg.):
   - J1: [-180, +180]
   - J2: [- 90, + 90]
   - J3: [-115, +115]

     Though J2 and J3 have different limits, but actually their limits are coupled with each other in the sense that the instantaneous limit of J2 or J3 joint must be computed as per the current position of the other one.
   - J4: [-130, +160]

     Cannot go to -180 and +180 deg. because of the excessive tension appearing in the connection wires.
   - J5: [-105, +180]

     Cannot go to +180 deg. because of the excessive tension appearing in the connection wires.
   - J6

     Cannot test since we don’t have any grippers yet.

## ROS2 Control

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

     **Edit A — line 11, add `UnlessCondition` to the imports:**
     - Original:
       ```python
       from launch.conditions import IfCondition
       ```
     - Edited:
       ```python
       from launch.conditions import IfCondition, UnlessCondition
       ```

     **Edit B — around line 234, add a `condition=` to the `ros2_control_node` Node so it only starts when the plugin is *not* `sim`:**
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

     **Edit A -- around line 36, immediately after the `tcgetattr(...)` if-block: add `cfmakeraw(&tty)`.**
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

## Expected Warnings & Errors at Launch

When running any of `ros2 launch pro_arm_moveit {move_arm, fake_arm_control, sim_arm_control, real_arm_control}.launch.py`, several warnings and `ERROR`-level messages appear in the terminal even though everything works -- RViz comes up and `Plan & Execute` moves the arm (real, fake, or simulated). Most are cosmetic upstream noise. Each item below is tagged with where it appears:
- **move** = `move_arm.launch.py`,
- **fake** = `fake_arm_control.launch.py`,
- **sim** = `sim_arm_control.launch.py`,
- **real** = `real_arm_control.launch.py`.

(Items not tagged with `sim` are absent in sim mode because the ROS2 Control edit in item 2 above skips the standalone `ros2_control_node` in sim mode.)

<!-- ### Safe to ignore (cosmetic / upstream noise) -->

1. **Deprecated `robot_description` parameter** _(move, fake, real)_
   ```
   [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
   ```
   The launch file passes `robot_description` as a parameter to `ros2_control_node`; the modern API expects it on the `~/robot_description` topic from `robot_state_publisher`. Controllers still load fine on Humble — purely a deprecation notice.

2. **FIFO realtime scheduling not permitted** _(move, fake, real)_
   ```
   [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling. 
   ```
   `ros2_control_node` tried to give itself realtime priority but the current user does not have `rtprio` limits configured. Has no impact in `fake` (or `sim`) mode. Worth fixing only when driving the real arm — add the user to a `realtime` group with a `/etc/security/limits.d/99-realtime.conf` entry.

3. **Octomap resolution not specified / no 3D sensor plugin defined** _(all 4)_
   ```
   [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
   [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
   ```
   MoveIt's octomap / sensor manager is initialized but no depth camera is configured. The arm has no 3D sensor, so the warning is expected. Can be silenced by removing the `sensors_3d.yaml` reference from the MoveIt config, but it is harmless.

4. **Deprecated `allow_nonzero_velocity_at_trajectory_end`** _(all 4)_
   ```
   [arm_trajectory_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
   ```
   `~/SES-P-ROS2-Arms/src/pro_arm_moveit/config/controllers_6dof.yaml` sets this flag; a future release of `joint_trajectory_controller` will default it to `false`. Pure deprecation notice.

5. **InteractiveMarkerDisplay namespace collision in RViz** _(all 4)_
   ```
   Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
   ```
   Known harmless issue on ROS 2 Humble — `rviz_default_plugins` is both linked into the `rviz2` executable and loaded again by `moveit_ros_visualization`. The second registration silently wins; nothing breaks. Upstream RViz / MoveIt issue, not specific to this workspace.

6. **`/recognize_objects` action server not available** _(all 4)_
   ```
   [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
   ```
   The MoveIt *Motion Planning* panel checks for an action server provided by ORK (Object Recognition Kitchen) for its "Detected Objects" tab. ORK is not installed (and is not packaged for ROS 2 Humble). The tab simply stays empty. Despite the `ERROR` severity in the log, it is harmless.

7. **Transient planning-scene update glitch** _(move, fake)_
   ```
   [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Maybe failed to update robot state, time diff: 0.069s
   ```
   Brief timing hiccup, usually during planning-scene initialization or on the first plan. Only worth investigating if it repeats continuously during normal operation.

8. **Controller update period slower than Gazebo physics step** _(sim)_
   ```
   [gz_ros2_control]:  Desired controller update period (0.0333333 s) is slower than the gazebo simulation period (0.001 s).
   ```
   Gazebo physics is stepping at 1 kHz but `controller_manager` is configured to tick at 30 Hz. The controllers are simply updated less often than physics — sim still runs consistently. Harmless.

9. **Transient `Failed to receive current joint state`** _(real)_
   ```
   [moveit_ros.trajectory_execution_manager]: Failed to receive current joint state
   ```
   Brief timing hiccup when MoveIt's trajectory execution manager asks for the latest joint state while the `pro_motor_hardware` read loop is mid-iteration on the LSS-P bus (each iteration polls all six servos sequentially). Harmless if it appears occasionally; only worth investigating if it repeats every iteration or blocks motion execution.