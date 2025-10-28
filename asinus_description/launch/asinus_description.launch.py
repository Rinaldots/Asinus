import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Helper to ensure config path uses the base robot name, not the namespaced one
    urdf_file_name = "asinus_2_wheel.urdf.xacro"
    pkg_asinus_description = get_package_share_directory("asinus_description")
    
    robot_description_path = os.path.join(pkg_asinus_description, "asinus", urdf_file_name)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(Command(['xacro ', robot_description_path]), value_type=str)}],
        remappings=[("/robot_description", "robot_description")],
    )

    rviz_config_file = os.path.join(pkg_asinus_description, "rviz", "diffbot_view.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])
