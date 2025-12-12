# Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland) & HarvestX Inc.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # -------------------------------------------------------------------------
    # 1. Configurações e Argumentos do Robô e Teleop (Combinados)
    # -------------------------------------------------------------------------

    # 1.1 Configurações do Robô (URDF e Controllers)
    urdf_file_name = "asinus_2_wheel.urdf.xacro"
    pkg_asinus_description = get_package_share_directory("asinus_description")
    robot_description_path = os.path.join(pkg_asinus_description, "asinus", urdf_file_name)

    # Gera a descrição do robô a partir do xacro
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', robot_description_path]), value_type=str)}
    robot_controllers = PathJoinSubstitution([FindPackageShare("asinus_demo_bringup"),"config","asinus_controllers_2_wheel.yaml",])
    
    # Arquivo de configuração Rviz (Opcional)
    rviz_config_file = PathJoinSubstitution([FindPackageShare("asinus_description"), "rviz", "diffbot_view.rviz"])

    # 1.2 Configurações de Teleop (Joystick)
    # Arquivo de configuração específico para PS4/PS3 (assumindo que ps4.config.yaml tem o mapeamento)
    teleop_twist_joy_config_file = PathJoinSubstitution([FindPackageShare("asinus_demo_bringup"), "config", "ps4.config.yaml",])
    
    # Argumentos de Launch (Do original 'ps3.launch.py')
    hw_type_arg = DeclareLaunchArgument(
        'hw_type', default_value=TextSubstitution(text='DualShock3'), description='Tipo de Hardware (DualShock3 ou DualShock4)')
    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value=TextSubstitution(text='diff_base_controller/cmd_vel_unstamped'), description='Tópico de saída para comando de velocidade')
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value=TextSubstitution(text='1.0'), description='Velocidade linear máxima')
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed', default_value=TextSubstitution(text='1.0'), description='Velocidade angular máxima')

    # -------------------------------------------------------------------------
    # 2. Definição dos Nós
    # -------------------------------------------------------------------------

    # 2.1 Nós de Controle e Estado do Robô (do 'asinus_launch.py')
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
        arguments=["diff_base_controller", "-c", "/controller_manager"]
    )

    # 2.2 Nós de Sensores (do 'asinus_launch.py')
    kinect_node = Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2",
                namespace="kinect",
            )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('cspc_lidar'),
                'launch',
                'lidar.launch.py'
            ])
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        # commentado para evitar falha em ambientes sem display X
        # launch_arguments: {'display': 'screen'} 
    )

    # 2.3 Container de Teleop (do 'ps3.launch.py')
    joy_container = ComposableNodeContainer(
        name='joy_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy',
                namespace='',
            ),
            ComposableNode(
                package='p9n_node',
                plugin='p9n_node::TeleopTwistJoyNode',
                name='teleop_twist_joy_node',
                namespace='',
                parameters=[
                    {'hw_type': LaunchConfiguration('hw_type')},
                    {'linear_speed': LaunchConfiguration('linear_speed')},
                    {'angular_speed': LaunchConfiguration('angular_speed')}
                ],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('topic_name'))
                ],
            )
        ],
    )
    
    # -------------------------------------------------------------------------
    # 3. Montagem do Launch Description
    # -------------------------------------------------------------------------

    ld = LaunchDescription()

    # Adicionar argumentos
    ld.add_action(hw_type_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(linear_speed_arg)
    ld.add_action(angular_speed_arg)

    # Adicionar nós de Controle e Estado
    ld.add_action(control_node)
    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)
    
    # Adicionar nós de Sensores
    ld.add_action(kinect_node)
    ld.add_action(lidar_launch)
    
    # Adicionar Teleop (Container de Joy/TeleopTwistJoy)
    ld.add_action(joy_container)
    
    # Adicionar RViz (Opcional, Removido do original para evitar falhas)
    # ld.add_action(rviz_node)

    return ld
