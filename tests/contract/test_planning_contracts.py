"""Contract tests for the real Planning node (uav_planning/real_planner),
exercised as a subprocess over real ROS 2 topics — same pattern as
tests/contract/test_node_contracts.py. No Gazebo/PX4 needed: Planning only
depends on Mission/LocalizationState/LocalMap/ObstacleSet, all synthesized
directly here. Run with: `pytest tests/contract -v`
"""
import math
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from uav_interfaces.msg import (
    Mission, Waypoint, LocalizationState, LocalMap, ObstacleSet, Obstacle,
    Trajectory, PlannerStatus,
)

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


def settle_and_clear(nodes, *collectors, seconds=SETTLE_SECONDS):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.05)
    for c in collectors:
        c.clear()


def make_empty_map(resolution=0.5, size=20, origin=(-5.0, -5.0, 0.0)):
    m = LocalMap()
    m.header.frame_id = 'map'
    m.resolution = resolution
    m.origin.x, m.origin.y, m.origin.z = origin
    m.size_x = m.size_y = m.size_z = size
    m.occupancy = [0] * (size * size * size)
    m.map_valid = True
    return m


def make_localization(x=0.0, y=0.0, z=1.0):
    loc = LocalizationState()
    loc.header.frame_id = 'map'
    loc.pose.position.x = x
    loc.pose.position.y = y
    loc.pose.position.z = z
    loc.pose.orientation.w = 1.0
    loc.localization_ok = True
    return loc


def make_mission(goal=(3.0, 0.0, 1.0), boundary_radius=50.0, max_speed=2.0):
    m = Mission()
    m.header.frame_id = 'map'
    m.mission_id = 'contract-test'
    wp = Waypoint()
    wp.position.x, wp.position.y, wp.position.z = goal
    wp.acceptance_radius = 0.5
    wp.waypoint_type = Waypoint.TYPE_TARGET
    m.waypoints = [wp]
    m.max_speed = max_speed
    m.boundary_radius = boundary_radius
    m.min_altitude = 0.0
    m.max_altitude = 20.0
    return m


def test_idle_without_inputs():
    planner = RunningNode('uav_planning', 'real_planner')
    watcher = Node('t_planning_idle')
    statuses = []
    watcher.create_subscription(
        PlannerStatus, '/planning/status', lambda m: statuses.append(m), SENSOR_QOS)

    settle_and_clear([watcher], statuses)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(watcher, timeout_sec=0.05)

    planner.stop()
    watcher.destroy_node()

    assert len(statuses) > 0, 'no PlannerStatus observed'
    assert all(s.state == PlannerStatus.STATE_IDLE for s in statuses)


def test_plans_straight_path_on_empty_map():
    planner = RunningNode('uav_planning', 'real_planner')
    driver = Node('t_planning_straight')
    mission_pub = driver.create_publisher(Mission, '/mission/current', MISSION_QOS)
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    trajectories = []
    statuses = []
    driver.create_subscription(
        Trajectory, '/planning/trajectory', lambda m: trajectories.append(m), SENSOR_QOS)
    driver.create_subscription(
        PlannerStatus, '/planning/status', lambda m: statuses.append(m), SENSOR_QOS)

    settle_and_clear([driver], trajectories, statuses)

    mission = make_mission(goal=(3.0, 0.0, 1.0))
    loc = make_localization(0.0, 0.0, 1.0)
    local_map = make_empty_map()

    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        mission_pub.publish(mission)
        loc_pub.publish(loc)
        map_pub.publish(local_map)
        rclpy.spin_once(driver, timeout_sec=0.05)

    planner.stop()
    driver.destroy_node()

    assert len(trajectories) > 0, 'no Trajectory observed'
    assert any(t.valid for t in trajectories), 'planner never produced a valid Trajectory'
    assert any(s.state == PlannerStatus.STATE_EXECUTING for s in statuses)

    last_valid = [t for t in trajectories if t.valid][-1]
    assert len(last_valid.points) >= 2
    end_point = last_valid.points[-1].position
    dist_to_goal = math.sqrt(
        (end_point.x - 3.0) ** 2 + (end_point.y - 0.0) ** 2 + (end_point.z - 1.0) ** 2)
    assert dist_to_goal < 1.0, f'trajectory endpoint too far from goal: {dist_to_goal}m'


def test_routes_around_obstacle_in_direct_path():
    planner = RunningNode('uav_planning', 'real_planner')
    driver = Node('t_planning_obstacle')
    mission_pub = driver.create_publisher(Mission, '/mission/current', MISSION_QOS)
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    obstacle_pub = driver.create_publisher(ObstacleSet, '/world_model/obstacles', SENSOR_QOS)
    trajectories = []
    driver.create_subscription(
        Trajectory, '/planning/trajectory', lambda m: trajectories.append(m), SENSOR_QOS)

    settle_and_clear([driver], trajectories)

    mission = make_mission(goal=(4.0, 0.0, 1.0))
    loc = make_localization(0.0, 0.0, 1.0)
    local_map = make_empty_map()

    obstacle_set = ObstacleSet()
    obstacle_set.header.frame_id = 'map'
    obstacle = Obstacle()
    obstacle.id = 1
    obstacle.position.x = 2.0
    obstacle.position.y = 0.0
    obstacle.position.z = 1.0
    obstacle.radius = 0.6
    obstacle.obstacle_class = Obstacle.CLASS_STATIC
    obstacle_set.obstacles = [obstacle]

    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        mission_pub.publish(mission)
        loc_pub.publish(loc)
        map_pub.publish(local_map)
        obstacle_pub.publish(obstacle_set)
        rclpy.spin_once(driver, timeout_sec=0.05)

    planner.stop()
    driver.destroy_node()

    valid_trajectories = [t for t in trajectories if t.valid]
    assert len(valid_trajectories) > 0, 'planner never produced a valid Trajectory around the obstacle'

    last = valid_trajectories[-1]
    for point in last.points:
        dist = math.sqrt(
            (point.position.x - 2.0) ** 2 +
            (point.position.y - 0.0) ** 2 +
            (point.position.z - 1.0) ** 2)
        assert dist > obstacle.radius, \
            f'trajectory point ({point.position.x},{point.position.y},{point.position.z}) is inside the obstacle'


def test_invalid_localization_produces_failed_status():
    planner = RunningNode('uav_planning', 'real_planner')
    driver = Node('t_planning_invalid_loc')
    mission_pub = driver.create_publisher(Mission, '/mission/current', MISSION_QOS)
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    map_pub = driver.create_publisher(LocalMap, '/world_model/local_map', SENSOR_QOS)
    statuses = []
    driver.create_subscription(
        PlannerStatus, '/planning/status', lambda m: statuses.append(m), SENSOR_QOS)

    settle_and_clear([driver], statuses)

    mission = make_mission()
    loc = make_localization()
    loc.localization_ok = False   # the failure condition under test
    local_map = make_empty_map()

    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        mission_pub.publish(mission)
        loc_pub.publish(loc)
        map_pub.publish(local_map)
        rclpy.spin_once(driver, timeout_sec=0.05)

    planner.stop()
    driver.destroy_node()

    assert len(statuses) > 0
    assert any(s.state == PlannerStatus.STATE_FAILED for s in statuses)
