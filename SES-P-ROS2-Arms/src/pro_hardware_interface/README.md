# LSS-P ROS2 Hardware Interface Package

The LSS-P-ROS2-Hardware repository contains the hardware interface for the Lynxmotion Smart Servos PRO (LSS-P) used to control the physical motors, as well as an example package to move a "RRBot" (2-joint robot).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Package installation](#package-installation)
- [Description package](#description-package)
- [Author](#author)
- [Resources](#resources)

## Prerequisites

1. [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
2. [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
3. ROS2 dev tools:
```
sudo apt install python3-pip
sudo pip install colcon-common-extensions
sudo pip install vcstool
```
4. Rosdep: Used to install dependencies when building from sources
```
sudo apt install python3-rosdep2
rosdep update
```

## Package installation

```
sudo apt install git
git clone https://github.com/Lynxmotion/LSS-P-ROS2-Hardware.git
mkdir -p src
mv LSS-P-ROS2-Hardware/* src
mv src LSS-P-ROS2-Hardware
```

### Install dependencies

```
cd LSS-P-ROS2-Hardware
rosdep install --from-path src -yi --rosdistro humble
```

### Build instructions

```
export GZ_VERSION=fortress
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Initialization instructions

You will have to source the workspace in every new session in which you wish to use these packages:

```
source install/setup.bash
```

## Description package

The description package contains the URDF description of the RRBot, or ''Revolute-Revolute Manipulator Robot'', which is a simple 3-linkage, 2-joint arm that we will use to demonstrate motor control.

**View Model in Rviz**

```
ros2 launch rrbot_example view_robot.launch.py
```

## RRBot example package (ros2_control)

This package contains the controller setup to move 2 PRO motors using a simple position controlled robot with one hardware interface.

Before controlling the real robot first follow these steps:

1. Update the firmware on the servos using [PRO Config](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-configuration-software/)

2. Follow the [initial setup](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-software/lss-flowarm/?#HInitialSetup) and make sure the IDs are configured correctly.

To control the servos:

1. Run the launch file

```
ros2 launch rrbot_example rrbot.launch.py
```

* Note: If the servos light up *Blue* they have been configured correctly if not try running:
```
sudo chmod 666 /dev/ttyACM0
```

2. You can test the forward position controller by opening another command interface and running the example:

```
source install/setup.bash
```
```
ros2 launch rrbot_example test_forward_position_controller.launch.py
```

3. To send a direct command use:

```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
  - 3.1
  - -3.1"
```
* Note: The units are in radians, the hardware interface converts it to 0.01° degrees which is the unit used on the LSS-PRO communication protocol. 


## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

If you want more details about the LSS-P communication protocol, visit this [website](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo-pro/lss-p-communication-protocol/).

Official Lynxmotion PRO Robotic Arm ROS2 Package available [here](https://github.com/Lynxmotion/SES-P-ROS2-Arms). 

Read more about the PRO Robotic Arm in the [Wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro-arms/).

Purchase the PRO arm on [RobotShop](https://www.robotshop.com/collections/lynxmotion-ses-pro-robotic-arms).

Have any questions? Ask them on the RobotShop [Community](https://community.robotshop.com/forum/c/lynxmotion/electronics-software/27).
