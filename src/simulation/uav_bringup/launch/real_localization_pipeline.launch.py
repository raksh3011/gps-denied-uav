"""Same pipeline as mock_pipeline.launch.py, except Localization is real:
LiDAR/IMU -> FAST-LIO2 -> lio_state_bridge, instead of MockLocalization.
Everything downstream (World Model, Mission, Planning, Safety, Vehicle) is
still mocked — this is the "swap one module for real, keep the rest mocked"
workflow described in docs/DEVELOPMENT.md.

Prerequisites this launch file does NOT start for you:
  1. Gazebo + PX4 SITL running, with a LiDAR/IMU-equipped vehicle model
     (docs/LOCALIZATION.md — coordinate the model with Person 4).
  2. simulation/launch/sensors_bridge.launch.py running, to bridge the
     simulated sensors into /lidar/points and /imu/data.
  3. The vendored FAST-LIO2 backend built (uav_localization.repos).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_localization_launch = PathJoinSubstitution(
        [FindPackageShare('uav_localization'), 'launch', 'real_localization.launch.py'])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(real_localization_launch)),
        Node(
            package='uav_world_model', executable='mock_world_model',
            name='mock_world_model', output='screen',
        ),
        Node(
            package='uav_mission', executable='mock_mission',
            name='mock_mission', output='screen',
        ),
        Node(
            package='uav_planning', executable='mock_planner',
            name='mock_planner', output='screen',
        ),
        Node(
            package='uav_safety', executable='mock_safety',
            name='mock_safety', output='screen',
        ),
        Node(
            package='uav_vehicle', executable='mock_vehicle',
            name='mock_vehicle', output='screen',
        ),
    ])
