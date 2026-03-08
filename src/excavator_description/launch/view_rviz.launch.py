#!/usr/bin/env python3
"""
Launch RViz + Joint State Publisher GUI for viewing the excavator model.
ROS 2 Jazzy
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    pkg_share = str(get_package_share_path("excavator_description"))
    default_model = os.path.join(pkg_share, "urdf", "excavator.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "config", "excavator_rviz_config.rviz")

    model_arg = DeclareLaunchArgument(
        "model", default_value=default_model, description="Path to URDF/Xacro file"
    )

    # Build robot_description using xacro (URDF resolves $(find ...) internally)
    xacro_command = Command([
        TextSubstitution(text="xacro "),
        LaunchConfiguration("model")
    ])
    robot_description = ParameterValue(xacro_command, value_type=str)

    # Joint State Publisher GUI needs robot_description to know the joints
    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Robot State Publisher publishes TF from joint states
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # RViz also needs robot_description locally (ROS 2 params are not global)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz],
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        joint_state_gui,
        rsp,
        rviz,
    ])

