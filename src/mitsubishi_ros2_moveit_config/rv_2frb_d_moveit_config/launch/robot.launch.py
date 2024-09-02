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
# Description: Launch a Mitsubishi RV-2FR robot with moveit config..
# https://adiro.com/en/
# https://www.software-byts.de/

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
        
    robot_ip = LaunchConfiguration("robot_ip")
    robot_udp_port = LaunchConfiguration('robot_udp_port')
    start_rviz = LaunchConfiguration('start_rviz')
    robot = LaunchConfiguration("robot")
    
    launch_arguments = {
        "use_sim": "False",
        "robot_ip": robot_ip,
        "robot_udp_port": robot_udp_port,
        "robot": robot
    }

    robot_value = robot.perform(context)

    robot_moveit_package_name = f"{robot_value}_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder(robot_value, package_name=robot_moveit_package_name)
        .robot_description(mappings=launch_arguments)
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(robot_moveit_package_name),
            "config",
            f"{robot_value}_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, robot_controllers],        
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mitsubishi_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{robot_value}_arm_controller", "-c", "/controller_manager"],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(robot_moveit_package_name), "rviz", "moveit.rviz"]
    )        
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )
    
    nodes_to_start = [        
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,       
        move_group_node,
        mitsubishi_arm_controller_spawner,
        static_tf_node                        
    ]

    return nodes_to_start

def generate_launch_description():
    
    declared_arguments = []        
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.20',   #old value 192.168.1.104
            description='Robot IP.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_udp_port',
            default_value='10000',
            description='Robot UDP port.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="robot",
            default_value="rv_2frb_d",
            description="Select the desired mitsubishi robot model",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])