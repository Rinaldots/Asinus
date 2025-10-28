# Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
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

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    urdf_file_name = "asinus_2_wheel.urdf.xacro"
    pkg_asinus_description = get_package_share_directory("asinus_description")
    robot_description_path = os.path.join(pkg_asinus_description, "asinus", urdf_file_name)

    # Generate robot_description from xacro and ensure it's passed as a string
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', robot_description_path]), value_type=str)}
    robot_controllers = PathJoinSubstitution([FindPackageShare("asinus_demo_bringup"),"config","asinus_controllers_2_wheel.yaml",])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("asinus_description"), "rviz", "diffbot_view.rviz"])

    teleop_twist_joy_config_file = PathJoinSubstitution([FindPackageShare("asinus_demo_bringup"), "config", "ps4.config.yaml",])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["asinus_base_controller", "-c", "/controller_manager"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 30.0
        }],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[teleop_twist_joy_config_file],
        remappings=[
            ('/cmd_vel', '/asinus_base_controller/cmd_vel_unstamped'),
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        # joy_node,
        # teleop_twist_joy_node,
    ])
