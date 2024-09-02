#!/usr/bin/python3
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
# Description: Launching a simulation envirement for a Mitsubishi robot.
# https://adiro.com/en/
# https://www.software-byts.de/

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    robot = LaunchConfiguration("robot")
    robot_value = robot.perform(context)
  
    
    # Set the path to the URDF file
    if robot_value:
        robot_srdf_file_name = f"{robot_value}.srdf"

    robot_moveit_package_name = f"{robot_value}_moveit_config"

    # Gazebo nodes     
    mitsubishi_ros2_gazebo = os.path.join(
        get_package_share_directory('mitsubishi_ros2_gazebo'),
        'worlds',
        'default.world')    
    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': mitsubishi_ros2_gazebo}.items(),)

    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_value],
                        output='screen')
    
    # Robot Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_moveit_package_name),"urdf", f"{robot_value}.urdf.xacro"]
            ),
            " ",
            "use_sim:=true",
            " "
            "robot_ip:=dont-care"
            " ",
            "robot_udp_port:=dont-care",
            " "
            "robot:=",
            robot,
            " "
        ]
    )
    robot_description = {'robot_description': robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    nodes_to_start = [
        gazebo_launch, 
        robot_state_publisher_node,
        spawn_entity_node
    ]
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot",
            default_value="rv_4frl_d",
            description="Select the desired mitsubishi robot model",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

   