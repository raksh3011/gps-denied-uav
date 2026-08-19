"""Contract tests that exercise the real (C++) mock nodes as subprocesses
over ROS 2 topics to verify producer/consumer compatibility, per
docs/TESTING.md. Requires the workspace to be built and sourced so
`ros2 run <pkg> <exe>` resolves. Run with: `pytest tests/contract -v`
"""
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import UInt32

from uav_interfaces.msg import (
    LocalizationState, Trajectory, TrajectoryPoint, VehicleCommand,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class RunningNode:
    """Launches a compiled mock node as a subprocess and tears it down."""

    def __init__(self, package: str, executable: str):
        self.proc = subprocess.Popen(
            ['ros2', 'run', package, executable],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def spin_for(nodes, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)


def test_safety_rejects_stale_or_invalid_and_holds():
    safety = RunningNode('uav_safety', 'mock_safety')
    watcher = Node('t_watcher_stale')
    cmd_msgs = []
    watcher.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    time.sleep(1.0)  # let the process come up
    spin_for([watcher], 1.0)

    safety.stop()
    watcher.destroy_node()

    assert len(cmd_msgs) > 0, 'no VehicleCommand observed from Safety'
    assert all(not m.valid for m in cmd_msgs)
    assert all(m.mode == VehicleCommand.MODE_HOLD for m in cmd_msgs)


def test_safety_forwards_valid_command_when_inputs_healthy():
    safety = RunningNode('uav_safety', 'mock_safety')
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

    time.sleep(1.0)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    assert any(m.valid for m in cmd_msgs), \
        'Safety must forward a valid VehicleCommand once loc+trajectory are healthy'


def test_vehicle_rejects_invalid_commands():
    vehicle = RunningNode('uav_vehicle', 'mock_vehicle')
    driver = Node('t_veh_driver')
    cmd_pub = driver.create_publisher(VehicleCommand, '/safety/vehicle_command', SENSOR_QOS)
    rejected = []
    accepted = []
    driver.create_subscription(
        UInt32, '/vehicle/rejected_count', lambda m: rejected.append(m.data), SENSOR_QOS)
    driver.create_subscription(
        UInt32, '/vehicle/accepted_count', lambda m: accepted.append(m.data), SENSOR_QOS)

    invalid = VehicleCommand()
    invalid.valid = False

    time.sleep(1.0)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        cmd_pub.publish(invalid)
        rclpy.spin_once(driver, timeout_sec=0.05)

    vehicle.stop()
    driver.destroy_node()

    assert len(rejected) > 0, 'MockVehicle never reported a rejected command'
    assert len(accepted) == 0, 'MockVehicle accepted an invalid command'
