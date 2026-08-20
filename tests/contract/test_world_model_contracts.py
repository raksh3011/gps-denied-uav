"""Contract tests for the real World Model node (uav_world_model/
real_world_model), exercised as a subprocess over real ROS 2 topics — same
pattern as test_planning_contracts.py. No Gazebo/PX4 needed: the node only
depends on a PointCloud2 (normally FAST-LIO2's /cloud_registered) and
LocalizationState, both synthesized directly here.
Run with: `pytest tests/contract -v`
"""
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from uav_interfaces.msg import LocalizationState, LocalMap, ObstacleSet

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
        # start_new_session: kill the ros2-run wrapper AND the node binary
        # together — see the identical comment in test_node_contracts.py.
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


def make_localization(x=0.0, y=0.0, z=1.0):
    loc = LocalizationState()
    loc.header.frame_id = 'map'
    loc.pose.position.x = x
    loc.pose.position.y = y
    loc.pose.position.z = z
    loc.pose.orientation.w = 1.0
    loc.localization_ok = True
    loc.confidence = 1.0
    return loc


def make_cloud(points):
    """Minimal XYZ float32 PointCloud2 from a list of (x, y, z) tuples."""
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    msg.is_dense = True
    return msg


def dense_blob(center, half_extent=0.4, step=0.1):
    """A solid cube of points around `center` — enough hits per voxel to
    cross the mapper's min_hits threshold, enough voxels to survive the
    min_cluster_voxels noise filter."""
    cx, cy, cz = center
    pts = []
    d = -half_extent
    while d <= half_extent:
        e = -half_extent
        while e <= half_extent:
            f = -half_extent
            while f <= half_extent:
                pts.append((cx + d, cy + e, cz + f))
                f += step
            e += step
        d += step
    return pts


def test_publishes_valid_local_map_with_pose():
    world_model = RunningNode('uav_world_model', 'real_world_model')
    driver = Node('t_wm_map')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    maps = []
    driver.create_subscription(
        LocalMap, '/world_model/local_map', lambda m: maps.append(m), SENSOR_QOS)

    settle_and_clear([driver], maps)

    loc = make_localization()
    end = time.monotonic() + 3.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        rclpy.spin_once(driver, timeout_sec=0.05)

    world_model.stop()
    driver.destroy_node()

    assert len(maps) > 0, 'no LocalMap observed'
    valid = [m for m in maps if m.map_valid]
    assert len(valid) > 0, 'LocalMap never became valid despite healthy localization'
    m = valid[-1]
    assert m.header.frame_id == 'map'
    assert m.resolution > 0.0
    assert len(m.occupancy) == m.size_x * m.size_y * m.size_z


def test_map_invalid_without_localization():
    world_model = RunningNode('uav_world_model', 'real_world_model')
    driver = Node('t_wm_invalid')
    maps = []
    driver.create_subscription(
        LocalMap, '/world_model/local_map', lambda m: maps.append(m), SENSOR_QOS)

    settle_and_clear([driver], maps)
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rclpy.spin_once(driver, timeout_sec=0.05)

    world_model.stop()
    driver.destroy_node()

    assert len(maps) > 0, 'no LocalMap observed'
    assert all(not m.map_valid for m in maps), \
        'map_valid must be false before any LocalizationState arrives'


def test_cloud_blob_becomes_occupancy_and_obstacle():
    world_model = RunningNode('uav_world_model', 'real_world_model')
    driver = Node('t_wm_obstacle')
    loc_pub = driver.create_publisher(LocalizationState, '/localization/state', SENSOR_QOS)
    cloud_pub = driver.create_publisher(PointCloud2, '/cloud_registered', SENSOR_QOS)
    maps = []
    obstacle_sets = []
    driver.create_subscription(
        LocalMap, '/world_model/local_map', lambda m: maps.append(m), SENSOR_QOS)
    driver.create_subscription(
        ObstacleSet, '/world_model/obstacles', lambda m: obstacle_sets.append(m), SENSOR_QOS)

    settle_and_clear([driver], maps, obstacle_sets)

    loc = make_localization()
    blob_center = (2.0, 0.0, 1.0)
    cloud = make_cloud(dense_blob(blob_center))

    end = time.monotonic() + 4.0
    while time.monotonic() < end:
        loc_pub.publish(loc)
        cloud_pub.publish(cloud)
        rclpy.spin_once(driver, timeout_sec=0.05)

    world_model.stop()
    driver.destroy_node()

    # Occupancy: the voxel containing the blob center must be occupied.
    valid_maps = [m for m in maps if m.map_valid]
    assert len(valid_maps) > 0
    m = valid_maps[-1]
    x = int((blob_center[0] - m.origin.x) / m.resolution)
    y = int((blob_center[1] - m.origin.y) / m.resolution)
    z = int((blob_center[2] - m.origin.z) / m.resolution)
    flat = x + y * m.size_x + z * m.size_x * m.size_y
    assert 0 <= flat < len(m.occupancy), 'blob center fell outside the published window'
    assert m.occupancy[flat] == 1, 'voxel at the blob center is not occupied'

    # Obstacles: at least one tracked obstacle near the blob center.
    populated = [s for s in obstacle_sets if len(s.obstacles) > 0]
    assert len(populated) > 0, 'no ObstacleSet with obstacles observed'
    obstacle = populated[-1].obstacles[0]
    dist = ((obstacle.position.x - blob_center[0]) ** 2 +
            (obstacle.position.y - blob_center[1]) ** 2 +
            (obstacle.position.z - blob_center[2]) ** 2) ** 0.5
    assert dist < 1.0, f'tracked obstacle centroid {dist:.2f}m from the blob center'
    assert obstacle.radius > 0.0
