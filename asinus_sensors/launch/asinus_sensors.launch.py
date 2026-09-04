# Stack de sensores + fusao rodando DIRETO no Orange Pi 3B.
#   ICM-20948 (I2C bus 2, pinos 3/5)  -> imu/data_raw + imu/mag
#   imu_filter_madgwick               -> imu/data  (remapeado p/ /imu, que o ekf.yaml espera)
#   GPS u-blox M8 (UART2 = ttyS2, pinos 8/10)  -> /fix
#   [use_fusion:=true] EKF + navsat_transform  p/ fundir odom + IMU + GPS
#
# Uso:
#   ros2 launch asinus_sensors asinus_sensors.launch.py
#   ros2 launch asinus_sensors asinus_sensors.launch.py gps_port:=/dev/ttyS2 use_fusion:=true
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sensors = FindPackageShare('asinus_sensors')
    pkg_desc = get_package_share_directory('asinus_description')

    navsat_params = PathJoinSubstitution([pkg_sensors, 'config', 'navsat_transform.yaml'])
    ekf_params = os.path.join(pkg_desc, 'config', 'ekf.yaml')

    gps_port = LaunchConfiguration('gps_port')
    gps_baud = LaunchConfiguration('gps_baud')
    use_fusion = LaunchConfiguration('use_fusion')

    return LaunchDescription([
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyS2',
                              description='UART do GPS = UART2 (ttyS2, pinos 8/10). Requer serial-getty@ttyS2 desativado. USB-TTL: /dev/ttyUSB0.'),
        DeclareLaunchArgument('gps_baud', default_value='9600'),
        DeclareLaunchArgument('use_fusion', default_value='false',
                              description="Se 'true', sobe tambem EKF + navsat_transform"),

        # IMU ICM-20948 -> imu/data_raw + imu/mag
        Node(
            package='asinus_sensors',
            executable='icm20948_node',
            name='icm20948',
            parameters=[{
                'i2c_bus': 2,          # I2C2 do RK3566 (pinos 3/5), overlay i2c2-m1. CONFIRMAR com `ls /dev/i2c-*` apos reboot
                'i2c_address': 0x69,
                'frame_id': 'imu',     # casa com o <link name="imu"> do URDF
                'frequency': 100.0,
            }],
            output='screen',
        ),

        # Fusao accel+gyro+mag -> orientacao. Publica em /imu (topico que o ekf.yaml usa)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': True,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[('imu/data', 'imu')],
            output='screen',
        ),

        # GPS NEO-6M -> /fix (sensor_msgs/NavSatFix)
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver',
            parameters=[{
                'port': gps_port,
                'baud': ParameterValue(gps_baud, value_type=int),
                'frame_id': 'gps_link',
            }],
            output='screen',
        ),

        # --- Fusao global opcional (use_fusion:=true) ---
        # EKF local (odom): funde a odometria das rodas + /imu. use_sim_time forcado False.
        Node(
            condition=IfCondition(use_fusion),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params, {'use_sim_time': False}],
            output='screen',
        ),
        # navsat_transform: usa /imu (heading) + /fix + odometry/filtered -> odometry/gps
        Node(
            condition=IfCondition(use_fusion),
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[navsat_params, {'use_sim_time': False}],
            remappings=[
                ('imu', 'imu'),
                ('gps/fix', 'fix'),
                ('odometry/filtered', 'odometry/filtered'),
            ],
            output='screen',
        ),
    ])
