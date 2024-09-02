#-- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2024 ADIRO Automatisierungstechnik GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#-- END LICENSE BLOCK ------------------------------------------------

# Author: Timo Schwarzer, Stefan Knoblauch
# Maintainer: Alexander Feuchter
# Last-Updated: January 22, 2024
# Description: Launch an mitsubishi robot
# https://adiro.com/en/
# https://www.software-byts.de/ 

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot")
    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_value = robot.perform(context)

    # Set the path to the URDF file
    if robot_value:
        robot_type_xacro_file_name = f"{robot_value}.xacro"

    # Load the robot description
    description_file_path = os.path.join(
        get_package_share_directory("mitsubishi_ros2_description"),
        "urdf",
        robot_value,
        robot_type_xacro_file_name,
    )    
    robot_description_config = xacro.process_file(description_file_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Launch RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("mitsubishi_ros2_description"),
                "rviz",
                "rviz_basic_settings.rviz",
            ),
        ],
        parameters=[robot_description],
    )

    nodes_to_start = [
        start_joint_state_publisher_node,
        start_joint_state_publisher_gui_node,
        robot_state_pub_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="gui",
            default_value="False",
            description="Flag to enable joint_state_publisher_gui",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_rviz", default_value="True", description="Whether to start RVIZ"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot",
            default_value="rv_2frb_d",
            description="Select the desired mitsubishi robot model",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
