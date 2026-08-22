"""Contract tests for the real Safety Supervisor node (uav_safety/
real_safety), exercised as a subprocess over real ROS 2 topics — same
pattern as test_planning_contracts.py / test_world_model_contracts.py.
No Gazebo/PX4 needed: real_safety only depends on topics synthesized
directly here.
Run with: `pytest tests/contract -v`
"""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import (
    LocalizationState, LocalMap, ObstacleSet, Obstacle, Trajectory,
    TrajectoryPoint, PlannerStatus, VehicleCommand, SystemHealth,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

SETTLE_SECONDS = 1.5


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class RunningNode:
    def __init__(self, package: str, executable: str, *args: str):
        self.proc = subprocess.Popen(
            ['ros2', 'run', package, executable, *args],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)

    def stop(self):
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait()


def settle_and_clear(nodes, *collectors, seconds=SETTLE_SECONDS):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)
    for c in collectors:
        c.clear()


def make_localization(status=LocalizationState.STATUS_NOMINAL, ok=True):
    loc = LocalizationState()
    loc.header.frame_id = 'map'
    loc.pose.position.x = 0.0
    loc.pose.position.y = 0.0
    loc.pose.position.z = 1.0
    loc.pose.orientation.w = 1.0
    loc.localization_ok = ok
    loc.status = status
    loc.confidence = 1.0 if ok else 0.0
    return loc


def make_trajectory(valid=True):
    traj = Trajectory()
    traj.header.frame_id = 'map'
    traj.valid = valid
    p1 = TrajectoryPoint()
    p1.position.x, p1.position.y, p1.position.z = 1.0, 0.0, 1.0
    p2 = TrajectoryPoint()
    p2.position.x, p2.position.y, p2.position.z = 3.0, 0.0, 1.0
    traj.points = [p1, p2]
    return traj


def make_map(valid=True):
    m = LocalMap()
    m.header.frame_id = 'map'
    m.resolution = 0.5
    m.size_x = m.size_y = m.size_z = 10
    m.occupancy = [0] * 1000
    m.map_valid = valid
    return m


def make_planner_status(state=PlannerStatus.STATE_EXECUTING):
    status = PlannerStatus()
    status.state = state
    return status


def test_forwards_valid_command_when_all_healthy():
    safety = RunningNode('uav_safety', 'real_safety')
    driver = Node('t_safety_healthy')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    status_pub = driver.create_publisher(PlannerStatus, '/planning/status', SENSOR_QOS)
    cmd_msgs = []
    health_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)
    driver.create_subscription(
        SystemHealth, '/safety/system_health', lambda m: health_msgs.append(m), SENSOR_QOS)

    settle_and_clear([driver], cmd_msgs, health_msgs)

    loc, traj, local_map, status = (
        make_localization(), make_trajectory(), make_map(), make_planner_status())
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        map_pub.publish(local_map)
        status_pub.publish(status)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    assert len(cmd_msgs) > 0, 'no VehicleCommand observed'
    valid = [c for c in cmd_msgs if c.valid]
    assert len(valid) > 0, 'Safety never forwarded a valid command despite healthy inputs'
    assert valid[-1].mode == VehicleCommand.MODE_POSITION

    assert len(health_msgs) > 0
    assert health_msgs[-1].overall_level == SystemHealth.LEVEL_OK


def test_degraded_localization_still_forwards_command():
    # Margasoochi (Planning) already responds to degraded localization; Safety
    # must defer, not double-block. See docs/SAFETY.md.
    safety = RunningNode('uav_safety', 'real_safety')
    driver = Node('t_safety_degraded')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    status_pub = driver.create_publisher(PlannerStatus, '/planning/status', SENSOR_QOS)
    cmd_msgs = []
    health_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)
    driver.create_subscription(
        SystemHealth, '/safety/system_health', lambda m: health_msgs.append(m), SENSOR_QOS)

    settle_and_clear([driver], cmd_msgs, health_msgs)

    loc = make_localization(status=LocalizationState.STATUS_DEGRADED, ok=True)
    traj, local_map, status = make_trajectory(), make_map(), make_planner_status()
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        map_pub.publish(local_map)
        status_pub.publish(status)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    valid = [c for c in cmd_msgs if c.valid]
    assert len(valid) > 0, 'DEGRADED localization must not block the command'
    assert any(h.overall_level == SystemHealth.LEVEL_WARN for h in health_msgs), \
        'DEGRADED localization must still be reflected in reported health'


def test_stale_localization_holds_with_invalid_command():
    # lost_hold_timeout_s pushed out of reach: this test isolates the
    # immediate hold behavior, not the sustained-loss LAND escalation
    # (covered separately by test_sustained_localization_loss_escalates_
    # to_land) — the node ticks continuously from startup, so the settle
    # period's elapsed time counts too and would otherwise trip the
    # default 3.0s threshold partway through this test's own window.
    safety = RunningNode(
        'uav_safety', 'real_safety',
        '--ros-args', '-p', 'lost_hold_timeout_s:=30.0')
    driver = Node('t_safety_stale')
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    cmd_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    settle_and_clear([driver], cmd_msgs)

    # Never publish LocalizationState at all -> immediately stale/absent.
    traj = make_trajectory()
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        traj_pub.publish(traj)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    assert len(cmd_msgs) > 0
    assert all(not c.valid for c in cmd_msgs), \
        'missing/stale LocalizationState must hold (valid=false), never forward a command'
    assert all(c.mode == VehicleCommand.MODE_HOLD for c in cmd_msgs)


def test_obstacle_on_trajectory_overrides_planner_valid_flag():
    # Independent defense-in-depth check: Planning claims valid=True, but
    # an obstacle sits directly on the trajectory's first point.
    safety = RunningNode('uav_safety', 'real_safety')
    driver = Node('t_safety_obstacle')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    obstacle_pub = driver.create_publisher(ObstacleSet, '/world_model/obstacles', SENSOR_QOS)
    status_pub = driver.create_publisher(PlannerStatus, '/planning/status', SENSOR_QOS)
    cmd_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    settle_and_clear([driver], cmd_msgs)

    loc, traj, local_map, status = (
        make_localization(), make_trajectory(), make_map(), make_planner_status())
    obstacle_set = ObstacleSet()
    obstacle_set.header.frame_id = 'map'
    obstacle = Obstacle()
    obstacle.id = 1
    obstacle.position.x, obstacle.position.y, obstacle.position.z = 1.0, 0.0, 1.0
    obstacle.radius = 0.5
    obstacle_set.obstacles = [obstacle]

    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        map_pub.publish(local_map)
        obstacle_pub.publish(obstacle_set)
        status_pub.publish(status)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    assert len(cmd_msgs) > 0
    assert all(not c.valid for c in cmd_msgs), \
        'an obstacle on the trajectory must force a hold even if Planning reports valid=True'


def test_sustained_localization_loss_escalates_to_land():
    safety = RunningNode(
        'uav_safety', 'real_safety',
        '--ros-args', '-p', 'lost_hold_timeout_s:=0.5')
    driver = Node('t_safety_sustained_loss')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    traj_pub = driver.create_publisher(Trajectory, '/planning/trajectory', SENSOR_QOS)
    cmd_msgs = []
    driver.create_subscription(
        VehicleCommand, '/safety/vehicle_command', lambda m: cmd_msgs.append(m), SENSOR_QOS)

    settle_and_clear([driver], cmd_msgs)

    loc = make_localization(status=LocalizationState.STATUS_LOST, ok=False)
    traj = make_trajectory()
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        traj_pub.publish(traj)
        rclpy.spin_once(driver, timeout_sec=0.05)

    safety.stop()
    driver.destroy_node()

    lands = [c for c in cmd_msgs if c.mode == VehicleCommand.MODE_LAND]
    assert len(lands) > 0, 'sustained LOST localization must eventually escalate to MODE_LAND'
    assert all(c.valid for c in lands), 'the LAND escalation must be an explicit valid command'
