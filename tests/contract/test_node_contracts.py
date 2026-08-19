"""Contract tests that exercise real mock nodes over ROS 2 topics to verify
producer/consumer compatibility, per docs/TESTING.md. Requires the workspace
to be built and sourced. Run with: `pytest tests/contract -v`
"""
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import (
    LocalizationState, Trajectory, TrajectoryPoint, VehicleCommand,
)
from uav_safety.mock_safety import MockSafety
from uav_vehicle.mock_vehicle import MockVehicle


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_for(nodes, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)


def test_safety_rejects_stale_or_invalid_and_holds():
    safety = MockSafety()
    pub = Node('t_pub')
    cmd_msgs = []
    pub.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    # No localization/trajectory published at all -> must hold, never claim valid.
    spin_for([safety, pub], 0.3)

    safety.destroy_node()
    pub.destroy_node()

    assert len(cmd_msgs) > 0
    assert all(not m.valid for m in cmd_msgs)
    assert all(m.mode == VehicleCommand.MODE_HOLD for m in cmd_msgs)


def test_safety_forwards_valid_command_when_inputs_healthy():
    safety = MockSafety()
    driver = Node('t_driver')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    cmd_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    loc = LocalizationState()
    loc.header.frame_id = 'map'
    loc.localization_ok = True
    traj = Trajectory()
    traj.header.frame_id = 'map'
    pt = TrajectoryPoint()
    pt.position.x = 5.0
    traj.points = [pt]
    traj.valid = True

    nodes = [safety, driver]
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)

    safety.destroy_node()
    driver.destroy_node()

    assert any(m.valid for m in cmd_msgs), \
        'Safety must forward a valid VehicleCommand once loc+trajectory are healthy'


def test_vehicle_rejects_invalid_commands():
    vehicle = MockVehicle()
    driver = Node('t_veh_driver')
    cmd_pub = driver.create_publisher(VehicleCommand, '/safety/vehicle_command', SENSOR_QOS)

    invalid = VehicleCommand()
    invalid.valid = False

    for _ in range(5):
        cmd_pub.publish(invalid)
        rclpy.spin_once(vehicle, timeout_sec=0.05)
        rclpy.spin_once(driver, timeout_sec=0.05)

    vehicle.destroy_node()
    driver.destroy_node()

    assert vehicle.rejected > 0
    assert vehicle.accepted == 0
