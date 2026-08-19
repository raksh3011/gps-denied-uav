"""Launches the fully mocked autonomy pipeline for Milestone 1:

Mission -> Global/Local Planner -> Localization -> World Model -> Safety -> PX4 Interface

No real algorithms are involved. This proves that every interface contract
is satisfiable end-to-end before any developer starts real implementation.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_localization', executable='mock_localization',
            name='mock_localization', output='screen',
        ),
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
