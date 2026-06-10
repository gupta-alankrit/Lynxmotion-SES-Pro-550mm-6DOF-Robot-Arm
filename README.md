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

## Repository Layout

| Path | Contents |
|------|----------|
| `assets/` | Firmware blobs and other media referenced by this README. |
| `SES-P-ROS2-Arms/` | A ready-to-build colcon workspace, named to match a fresh clone of the official Lynxmotion [`SES-P-ROS2-Arms`](https://github.com/Lynxmotion/SES-P-ROS2-Arms) repo. Contains those upstream packages with the patches from [`SES-P-ROS2-Arms/README.md`](SES-P-ROS2-Arms/README.md#ros2-control) already applied, plus a custom `machine_vision_pkg/` for vision-driven manipulation. See that README for the build, run, and per-node documentation. |
| `README.md` | This file -- joint limits and known launch warnings. |

If you'd rather start from a clean upstream clone, ignore `SES-P-ROS2-Arms/` and follow the patches in [`SES-P-ROS2-Arms/README.md#ros2-control`](SES-P-ROS2-Arms/README.md#ros2-control) by hand instead.

## Cloning & Setup

Two ways to get the workspace onto your machine, depending on whether you already maintain a colcon workspace.

### Option A -- clone the whole repo and build in place

Cleanest if you don't have an existing ROS 2 workspace, or want the documentation alongside the code.

```bash
git clone https://github.com/gupta-alankrit/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm.git
cd Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms
rosdep install --from-paths src --ignore-src -r -y     # one-time
colcon build --symlink-install
source install/setup.bash
```

You can now run any of the `ros2 launch machine_vision_pkg ...` commands from inside `SES-P-ROS2-Arms/`.

### Option B -- drop the packages into your existing workspace

Use this if you already have a colcon workspace at, say, `~/ros2_ws/`. Either copy the package directories in:

```bash
git clone https://github.com/gupta-alankrit/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm.git /tmp/lynx-doc-repo
cp -r /tmp/lynx-doc-repo/SES-P-ROS2-Arms/src/* ~/SES-P-ROS2-Arms/src/
cd ~/SES-P-ROS2-Arms
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

… or, if you only want a subset (e.g. just the custom vision package because you already have the upstream `pro_*` packages), replace the `cp -r` line with the specific directory you want, for instance:

```bash
cp -r /tmp/lynx-doc-repo/SES-P-ROS2-Arms/src/machine_vision_pkg ~/ros2_ws/src/
```

Detailed per-node and per-launch-file documentation lives in [`SES-P-ROS2-Arms/README.md`](SES-P-ROS2-Arms/README.md).

## Expected Warnings & Errors at Launch

When running any of `ros2 launch pro_arm_moveit {move_arm, fake_arm_control, sim_arm_control, real_arm_control}.launch.py`, several warnings and `ERROR`-level messages appear in the terminal even though everything works -- RViz comes up and `Plan & Execute` moves the arm (real, fake, or simulated). Most are cosmetic upstream noise. Each item below is tagged with where it appears:
- **move** = `move_arm.launch.py`,
- **fake** = `fake_arm_control.launch.py`,
- **sim** = `sim_arm_control.launch.py`,
- **real** = `real_arm_control.launch.py`.

(Items not tagged with `sim` are absent in sim mode because [item 2 of the workspace's ROS2 Control patches](SES-P-ROS2-Arms/README.md#ros2-control) skips the standalone `ros2_control_node` in sim mode.)

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
