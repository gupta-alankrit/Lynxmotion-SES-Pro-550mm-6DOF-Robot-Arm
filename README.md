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
   - J5: [-180, +105]

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
1. The `move_arm.launch.py` (which is included by `sim_arm_control.launch.py`) still unconditionally starts a standalone `ros2_control_node`. In `sim` mode this node crashes because `gz_ros2_control` (running inside the Gazebo process) already hosts its own `controller_manager` and owns the `ign_ros2_control/IgnitionSystem` hardware interface; the standalone node can't load that class because it filters by base type `hardware_interface::SystemInterface`. The crash looks like:
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
     - `sim` (`sim_arm_control.launch.py`) → standalone node is skipped; gz_ros2_control's in-process
  controller_manager handles everything.
     - `real` (`real_arm_control.launch.py`) → standalone node still starts and loads 
  `pro_motor_hardware/ProMotorHardware` to talk to the servos.
