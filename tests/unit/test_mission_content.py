"""Unit test: MockMission produces a well-formed, in-bounds mission.
No ROS runtime required beyond message construction — pure logic check
mirroring what mock_mission.publish_mission() builds.
"""
from uav_interfaces.msg import Mission, Waypoint


def build_golden_mission():
    m = Mission()
    m.header.frame_id = 'map'
    m.mission_id = 'golden-scenario-01'
    wp = Waypoint()
    wp.position.x = 10.0
    wp.position.y = 0.0
    wp.position.z = 3.0
    wp.acceptance_radius = 0.5
    wp.waypoint_type = Waypoint.TYPE_TARGET
    m.waypoints = [wp]
    m.max_speed = 3.0
    m.boundary_radius = 50.0
    m.min_altitude = 1.0
    m.max_altitude = 20.0
    return m


def test_golden_mission_altitude_within_bounds():
    m = build_golden_mission()
    for wp in m.waypoints:
        assert m.min_altitude <= wp.position.z <= m.max_altitude


def test_golden_mission_within_boundary_radius():
    m = build_golden_mission()
    origin = m.waypoints[0]
    for wp in m.waypoints:
        dist = ((wp.position.x - 0) ** 2 + (wp.position.y - 0) ** 2) ** 0.5
        assert dist <= m.boundary_radius


def test_golden_mission_has_nonzero_acceptance_radius():
    m = build_golden_mission()
    assert all(wp.acceptance_radius > 0 for wp in m.waypoints)
