"""Integration test: brings up the entire mocked pipeline (Mission -> Planner
-> Localization -> World Model -> Safety -> Vehicle) as real nodes and
verifies a VehicleCommand eventually flows out the other end. This is the
automated version of Milestone 1's "see all modules communicate correctly".
Run with: `pytest tests/integration -v` (workspace must be built/sourced).
"""
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import VehicleCommand
from uav_localization.mock_localization import MockLocalization
from uav_world_model.mock_world_model import MockWorldModel
from uav_mission.mock_mission import MockMission
from uav_planning.mock_planner import MockPlanner
from uav_safety.mock_safety import MockSafety
from uav_vehicle.mock_vehicle import MockVehicle

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_full_mock_pipeline_produces_vehicle_command():
    nodes = [
        MockLocalization(), MockWorldModel(), MockMission(),
        MockPlanner(), MockSafety(), MockVehicle(),
    ]
    watcher = Node('t_watcher')
    commands = []
    watcher.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: commands.append(m), SENSOR_QOS)
    nodes.append(watcher)

    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)

    for n in nodes:
        n.destroy_node()

    assert len(commands) > 0, 'no VehicleCommand observed within timeout'
    assert any(m.valid for m in commands), \
        'pipeline never reached a valid VehicleCommand end-to-end'
