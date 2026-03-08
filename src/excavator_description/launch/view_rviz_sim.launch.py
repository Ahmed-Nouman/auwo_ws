#!/usr/bin/env python3
"""
Launch RViz for viewing the excavator model driven by Gazebo.
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
    default_rviz  = os.path.join(pkg_share, "config", "excavator_rviz_config.rviz")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model,
        description="Path to URDF/Xacro file",
    )

    # Build robot_description from xacro (for RViz only)
    xacro_command = Command([
        TextSubstitution(text="xacro "),
        LaunchConfiguration("model")
    ])
    robot_description = ParameterValue(xacro_command, value_type=str)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz],
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    return LaunchDescription([
        model_arg,
        rviz,
    ])

