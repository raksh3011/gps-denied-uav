"""Top-level launch entry point. Delegates to uav_bringup so `ros2 launch`
works from a predictable repo-relative path in addition to the package name.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    uav_bringup_launch = PathJoinSubstitution(
        [FindPackageShare('uav_bringup'), 'launch', 'mock_pipeline.launch.py'])
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(uav_bringup_launch)),
    ])
