#!/usr/bin/env -S ros2 launch
"""Bring up the real pro_arm + RealSense + classifier + reprojection, and run
the vision-driven pick-and-place loop."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")

    drop_x = LaunchConfiguration("drop_x")
    drop_y = LaunchConfiguration("drop_y")
    drop_z = LaunchConfiguration("drop_z")
    drop_roll = LaunchConfiguration("drop_roll")
    drop_pitch = LaunchConfiguration("drop_pitch")
    drop_yaw = LaunchConfiguration("drop_yaw")

    target_offset_z = LaunchConfiguration("target_offset_z")
    target_roll = LaunchConfiguration("target_roll")
    target_pitch = LaunchConfiguration("target_pitch")
    target_yaw = LaunchConfiguration("target_yaw")

    wait_at_target_sec = LaunchConfiguration("wait_at_target_sec")
    wait_at_drop_sec = LaunchConfiguration("wait_at_drop_sec")
    settle_sec = LaunchConfiguration("settle_sec")
    detection_timeout_sec = LaunchConfiguration("detection_timeout_sec")

    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    planning_time = LaunchConfiguration("planning_time")
    startup_delay = LaunchConfiguration("startup_delay")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    real_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("pro_arm_moveit"),
                    "launch",
                    "real_arm_control.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("dof", dof),
            ("size", size),
            ("use_sim_time", use_sim_time),
            ("log_level", log_level),
        ],
    )

    objects_3d = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("machine_vision_pkg"),
                    "launch",
                    "objects_in_3D.launch.xml",
                ]
            )
        ),
    )

    classify_node = Node(
        package="machine_vision_pkg",
        executable="classify_pick_and_place",
        name="classify_pick_and_place",
        output="screen",
        parameters=[
            {
                "drop_x": drop_x,
                "drop_y": drop_y,
                "drop_z": drop_z,
                "drop_roll": drop_roll,
                "drop_pitch": drop_pitch,
                "drop_yaw": drop_yaw,
                "target_offset_z": target_offset_z,
                "target_roll": target_roll,
                "target_pitch": target_pitch,
                "target_yaw": target_yaw,
                "wait_at_target_sec": wait_at_target_sec,
                "wait_at_drop_sec": wait_at_drop_sec,
                "settle_sec": settle_sec,
                "detection_timeout_sec": detection_timeout_sec,
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
                "planning_time": planning_time,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    delayed_classify_node = TimerAction(period=startup_delay, actions=[classify_node])

    return LaunchDescription(
        declared_arguments + [real_arm, objects_3d, delayed_classify_node]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "dof", default_value="6", choices=["5", "6"],
            description="Degrees of freedom of the pro_arm.",
        ),
        DeclareLaunchArgument(
            "size", default_value="550", choices=["550", "900"],
            description="Size variant of the pro_arm.",
        ),
        DeclareLaunchArgument(
            "drop_x", default_value="0.0",
            description="Drop point X in pro_arm_base_link [m].",
        ),
        DeclareLaunchArgument(
            "drop_y", default_value="-0.3",
            description="Drop point Y in pro_arm_base_link [m].",
        ),
        DeclareLaunchArgument(
            "drop_z", default_value="0.15",
            description="Drop point Z in pro_arm_base_link [m].",
        ),
        DeclareLaunchArgument(
            "drop_roll", default_value="0.0",
            description="EE roll at the drop point [rad].",
        ),
        DeclareLaunchArgument(
            "drop_pitch", default_value="0.0",
            description="EE pitch at the drop point [rad].",
        ),
        DeclareLaunchArgument(
            "drop_yaw", default_value="0.0",
            description="EE yaw at the drop point [rad].",
        ),
        DeclareLaunchArgument(
            "target_offset_z", default_value="0.1",
            description="Hover height above each detected target [m].",
        ),
        DeclareLaunchArgument(
            "target_roll", default_value="0.0",
            description="EE roll when hovering above a target [rad].",
        ),
        DeclareLaunchArgument(
            "target_pitch", default_value="0.0",
            description="EE pitch when hovering above a target [rad].",
        ),
        DeclareLaunchArgument(
            "target_yaw", default_value="0.0",
            description="EE yaw when hovering above a target [rad].",
        ),
        DeclareLaunchArgument(
            "wait_at_target_sec", default_value="5.0",
            description="Seconds to dwell above the target before moving to drop.",
        ),
        DeclareLaunchArgument(
            "wait_at_drop_sec", default_value="5.0",
            description="Seconds to dwell at the drop point before returning home.",
        ),
        DeclareLaunchArgument(
            "settle_sec", default_value="2.0",
            description="Seconds to let the temporal tracker mature after enabling the classifier.",
        ),
        DeclareLaunchArgument(
            "detection_timeout_sec", default_value="10.0",
            description="Maximum seconds to wait for one /reprojection_3D/targets message after the settle window.",
        ),
        DeclareLaunchArgument(
            "velocity_scaling", default_value="0.5",
            description="Max joint velocity scaling in (0, 1]; keep low on the real arm.",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling", default_value="0.5",
            description="Max joint acceleration scaling in (0, 1].",
        ),
        DeclareLaunchArgument(
            "planning_time", default_value="10.0",
            description="Seconds OMPL is allowed per planning request.",
        ),
        DeclareLaunchArgument(
            "startup_delay", default_value="15.0",
            description="Seconds to wait for move_group, camera, and classifier before starting the loop.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Set true only when driving a simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="warn",
            description="Log level for ROS 2 nodes brought up by this launch file.",
        ),
    ]
