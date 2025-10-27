#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import os

def generate_launch_description():
    
    # Package directories
    pkg_asinus_slam = FindPackageShare('asinus_slam').find('asinus_slam')
    pkg_asinus_gz = FindPackageShare('asinus_gz').find('asinus_gz')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode'
    )

    
    # Configuration files
    rtabmap_config = os.path.join(pkg_asinus_slam, 'config', 'rtabmap_optimized.yaml')
    
    remappings = [
        ('rgb/image', '/kinect_rgb/image_raw'),
        ('depth/image', '/kinect_depth/image_raw'),
        ('rgb/camera_info', '/kinect_rgb/camera_info'),
        ('depth/camera_info', '/kinect_depth/camera_info'),
        ('scan', '/scan'),
        ('odom', '/odom'),
        ('gps/fix', '/navsat'),
    ]
    namespace = LaunchConfiguration('namespace', default='')

    rtabmap_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        arguments=['-d'],
        parameters=[rtabmap_config],
        remappings=remappings,
        namespace=namespace)

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtabmap_config],
        arguments=['-d'],
        namespace=namespace,
        condition=UnlessCondition(LaunchConfiguration('localization')),
        remappings=remappings)

    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[rtabmap_config],
        namespace=namespace,
        remappings=remappings)
        
    rtabmap_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[rtabmap_config, {'approx_sync_max_interval': 0.02}],
        namespace=namespace,
        remappings=remappings)
        
    # RTABMap localization node (when localization=true)
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_slam',
        output='screen',
        parameters=[rtabmap_config, {
            'Mem/IncrementalMemory': False,
            'Mem/InitWMWithAllNodes': True,
        }],
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    
        


    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        rtabmap_odom,
        rtabmap_sync,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
    ])
