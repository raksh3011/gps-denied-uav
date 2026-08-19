"""Real Localization: LiDAR/IMU -> FAST-LIO2 -> lio_state_bridge -> LocalizationState.

Requires the vendored LIO backend to be present (see ../../../uav_localization.repos
and docs/LOCALIZATION.md — `vcs import src < uav_localization.repos` then
`colcon build`). This launch file assumes the backend package is named
`fast_lio` and exposes a `mapping.launch.py` that accepts a `config_file`
argument, matching the common ROS 2 ports of FAST-LIO2 as of Jazzy. If your
vendored fork differs, adjust the IncludeLaunchDescription below and note the
change in docs/LOCALIZATION.md.

Publishes static transforms for lidar_link and imu_link -> base_link per
docs/CONVENTIONS.md. The offsets below are placeholders — Person 1 owns
measuring/calibrating the real sensor mount and updating them (and the
matching entries in config/fast_lio_x500.yaml) once the sensor mount is
finalized with Person 4.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fast_lio_config = PathJoinSubstitution(
        [FindPackageShare('uav_localization'), 'config', 'fast_lio_x500.yaml'])
    fast_lio_launch = PathJoinSubstitution(
        [FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'])

    return LaunchDescription([
        # lidar_link -> base_link (placeholder offset, see docstring above)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='lidar_link_tf', output='screen',
            arguments=['0.05', '0', '0.10', '0', '0', '0', 'base_link', 'lidar_link'],
        ),
        # imu_link -> base_link (placeholder offset, see docstring above)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='imu_link_tf', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'config_file': fast_lio_config}.items(),
        ),
        Node(
            package='uav_localization', executable='lio_state_bridge',
            name='lio_state_bridge', output='screen',
            parameters=[{
                'odometry_topic': '/Odometry',
                'map_frame': 'map',
                'staleness_timeout_s': 0.3,
                'lost_timeout_s': 1.5,
                'publish_rate_hz': 100.0,
            }],
        ),
    ])
