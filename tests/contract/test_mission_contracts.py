"""Contract tests for the real Mission node (uav_mission/real_mission),
exercised as a subprocess over real ROS 2 topics — same pattern as the
other test_*_contracts.py files. No Gazebo/PX4 needed.
Run with: `pytest tests/contract -v`
"""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

from uav_interfaces.msg import Mission, LocalizationState

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
MISSION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

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


def make_localization(x, y, z, ok=True):
    loc = LocalizationState()
    loc.header.frame_id = 'map'
    loc.pose.position.x = x
    loc.pose.position.y = y
    loc.pose.position.z = z
    loc.pose.orientation.w = 1.0
    loc.localization_ok = ok
    return loc


def test_publishes_default_golden_scenario():
    mission = RunningNode('uav_mission', 'real_mission')
    watcher = Node('t_mission_default')
    missions = []
    watcher.create_subscription(
        Mission, '/mission/current', lambda m: missions.append(m), MISSION_QOS)

    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(watcher, timeout_sec=0.05)

    mission.stop()
    watcher.destroy_node()

    assert len(missions) > 0, 'no Mission observed'
    m = missions[-1]
    assert len(m.waypoints) == 1
    assert m.waypoints[0].position.x == pytest.approx(10.0)
    assert m.waypoints[0].position.z == pytest.approx(3.0)
    assert m.max_speed == pytest.approx(3.0)


def test_advances_through_multi_leg_mission_as_waypoints_reached():
    mission = RunningNode(
        'uav_mission', 'real_mission',
        '--ros-args',
        '-p', 'waypoint_x:=[3.0,8.0]',
        '-p', 'waypoint_y:=[0.0,0.0]',
        '-p', 'waypoint_z:=[1.0,1.0]',
        '-p', 'waypoint_yaw:=[0.0,0.0]',
        '-p', 'waypoint_acceptance_radius:=[0.5,0.5]',
        '-p', 'waypoint_type:=[0,1]')
    driver = Node('t_mission_multileg')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    missions = []
    driver.create_subscription(
        Mission, '/mission/current', lambda m: missions.append(m), MISSION_QOS)

    settle_and_clear([driver], missions)

    # Not yet at leg 1 -> still 2 waypoints.
    far = make_localization(0.0, 0.0, 1.0)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        loc_pub.publish(far)
        rclpy.spin_once(driver, timeout_sec=0.05)
    assert len(missions) > 0
    assert len(missions[-1].waypoints) == 2

    # Reach leg 1 (3.0, 0.0, 1.0) -> should advance to leg 2.
    at_leg1 = make_localization(3.0, 0.0, 1.0)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        loc_pub.publish(at_leg1)
        rclpy.spin_once(driver, timeout_sec=0.05)
    assert len(missions[-1].waypoints) == 1
    assert missions[-1].waypoints[0].position.x == pytest.approx(8.0)

    # Reach leg 2 (8.0, 0.0, 1.0) -> mission complete, empty waypoints.
    at_leg2 = make_localization(8.0, 0.0, 1.0)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        loc_pub.publish(at_leg2)
        rclpy.spin_once(driver, timeout_sec=0.05)

    mission.stop()
    driver.destroy_node()

    assert len(missions[-1].waypoints) == 0


def test_unhealthy_localization_does_not_advance_mission():
    mission = RunningNode('uav_mission', 'real_mission')
    driver = Node('t_mission_unhealthy')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    missions = []
    driver.create_subscription(
        Mission, '/mission/current', lambda m: missions.append(m), MISSION_QOS)

    settle_and_clear([driver], missions)

    # Physically at the goal, but localization_ok=False.
    at_goal_bad_loc = make_localization(10.0, 0.0, 3.0, ok=False)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        loc_pub.publish(at_goal_bad_loc)
        rclpy.spin_once(driver, timeout_sec=0.05)

    mission.stop()
    driver.destroy_node()

    assert len(missions) > 0
    assert len(missions[-1].waypoints) == 1, \
        'an untrustworthy position must never advance the mission'
