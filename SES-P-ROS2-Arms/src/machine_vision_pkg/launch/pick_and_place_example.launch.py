#!/usr/bin/env -S ros2 launch
"""Bring up the real pro_arm with MoveIt and run the example pick-and-place sequence."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
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

    example_node = Node(
        package="machine_vision_pkg",
        executable="pick_and_place_example",
        name="pick_and_place_example",
        output="screen",
        parameters=[
            {
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
                "planning_time": planning_time,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    delayed_example_node = TimerAction(period=startup_delay, actions=[example_node])

    return LaunchDescription(declared_arguments + [real_arm, delayed_example_node])


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "dof",
            default_value="6",
            choices=["5", "6"],
            description="Degrees of freedom of the pro_arm.",
        ),
        DeclareLaunchArgument(
            "size",
            default_value="550",
            choices=["550", "900"],
            description="Size variant of the pro_arm.",
        ),
        DeclareLaunchArgument(
            "velocity_scaling",
            default_value="0.3",
            description="Max joint velocity scaling in (0, 1]; keep low on the real arm.",
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.3",
            description="Max joint acceleration scaling in (0, 1].",
        ),
        DeclareLaunchArgument(
            "planning_time",
            default_value="10.0",
            description="Seconds OMPL is allowed per planning request.",
        ),
        DeclareLaunchArgument(
            "startup_delay",
            default_value="12.0",
            description="Seconds to wait for move_group and controllers before starting the pick-and-place example node.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true only when driving a simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log level for ROS 2 nodes brought up by this launch file.",
        ),
    ]
