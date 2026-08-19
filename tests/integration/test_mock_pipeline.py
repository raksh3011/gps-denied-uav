"""Integration test: brings up the entire mocked pipeline (Mission -> Planner
-> Localization -> World Model -> Safety -> Vehicle) as real C++ processes
and verifies a VehicleCommand eventually flows out the other end. This is
the automated version of Milestone 1's "see all modules communicate
correctly". Run with: `pytest tests/integration -v` (workspace must be
built/sourced so `ros2 run` resolves each executable).
"""
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import VehicleCommand

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

PIPELINE_NODES = [
    ('uav_localization', 'mock_localization'),
    ('uav_world_model', 'mock_world_model'),
    ('uav_mission', 'mock_mission'),
    ('uav_planning', 'mock_planner'),
    ('uav_safety', 'mock_safety'),
    ('uav_vehicle', 'mock_vehicle'),
]


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_full_mock_pipeline_produces_vehicle_command():
    procs = [
        subprocess.Popen(
            ['ros2', 'run', pkg, exe],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for pkg, exe in PIPELINE_NODES
    ]

    watcher = Node('t_watcher')
    commands = []
    watcher.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: commands.append(m), SENSOR_QOS)

    try:
        time.sleep(2.0)  # let every process come up and start publishing
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            rclpy.spin_once(watcher, timeout_sec=0.05)
    finally:
        watcher.destroy_node()
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    assert len(commands) > 0, 'no VehicleCommand observed within timeout'
    assert any(m.valid for m in commands), \
        'pipeline never reached a valid VehicleCommand end-to-end'
