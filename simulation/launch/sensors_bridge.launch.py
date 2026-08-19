"""Bridges the simulated LiDAR + IMU from Gazebo into ROS 2 topics, per
uav_localization/config/ros_gz_bridge_lidar_imu.yaml. Requires a running
Gazebo world with a LiDAR/IMU-equipped vehicle model (see
docs/LOCALIZATION.md for the current placeholder model-name assumption and
what Person 4 needs to finalize on the vehicle/world side).
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('uav_localization'), 'config', 'ros_gz_bridge_lidar_imu.yaml'])

    return LaunchDescription([
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='lidar_imu_bridge', output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
    ])
