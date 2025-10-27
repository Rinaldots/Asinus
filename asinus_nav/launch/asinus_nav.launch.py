#!/usr/bin/env python3

# Copyright 2023 Rinaldo T.S Filho
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Whether to use simulation or real robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "map_file",
            default_value="",
            description="Path to the map file to use for navigation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_slam",
            default_value="false",
            description="Whether to use SLAM for mapping",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz",
        )
    )

    # Initialize arguments
    use_sim = LaunchConfiguration("use_sim")
    map_file = LaunchConfiguration("map_file")
    use_slam = LaunchConfiguration("use_slam")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get package paths
    asinus_nav_share = FindPackageShare("asinus_nav")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    # Navigation parameters
    nav2_params_file = PathJoinSubstitution([
        asinus_nav_share,
        "config",
        "nav2_params.yaml"
    ])

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        asinus_nav_share,
        "rviz",
        "asinus_nav.rviz"
    ])

    # Include robot launch (simulation or real)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("asinus_gz"),
                "launch",
                "asinus_one_robot.launch.py"
            ])
        ]),
        condition=IfCondition(use_sim)
    )

    # Include Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share,
                "launch",
                "bringup_launch.py"
            ])
        ]),
        launch_arguments={
            "params_file": nav2_params_file,
            "map": map_file,
            "use_sim_time": use_sim,
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                nav2_bringup_share,
                "launch",
                "slam_launch.py"
            ])
        ]),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": use_sim,
        }.items(),
        condition=IfCondition(use_slam)
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim}],
        output="screen",
        condition=IfCondition(use_rviz)
    )

    # Create launch description
    nodes = [
        robot_launch,
        nav2_launch,
        slam_launch,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
