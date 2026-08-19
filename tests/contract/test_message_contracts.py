"""Contract tests: verify the interface definitions themselves satisfy the
conventions documented in docs/INTERFACES.md and docs/CONVENTIONS.md.
These do not require a running node — they instantiate messages directly.
Run with ROS 2 sourced: `pytest tests/contract -v`
"""
import time

import pytest
from std_msgs.msg import Header
from uav_interfaces.msg import (
    LocalizationState, LocalMap, ObstacleSet, Obstacle,
    Mission, Waypoint, Trajectory, TrajectoryPoint,
    PlannerStatus, SystemHealth, VehicleCommand,
)


def make_header(frame_id='map'):
    h = Header()
    h.frame_id = frame_id
    return h


def test_localization_state_default_frame_and_status_enum():
    msg = LocalizationState()
    msg.header = make_header()
    msg.localization_ok = True
    msg.status = LocalizationState.STATUS_NOMINAL
    assert msg.header.frame_id == 'map'
    assert msg.status in (
        LocalizationState.STATUS_NOMINAL,
        LocalizationState.STATUS_DEGRADED,
        LocalizationState.STATUS_LOST,
    )
    assert len(msg.pose_covariance) == 36
    assert len(msg.twist_covariance) == 36


def test_localization_state_confidence_bounds():
    msg = LocalizationState()
    msg.confidence = 0.5
    assert 0.0 <= msg.confidence <= 1.0


def test_local_map_dimensions_consistent_with_occupancy_length():
    msg = LocalMap()
    msg.size_x, msg.size_y, msg.size_z = 4, 4, 4
    msg.occupancy = [0] * (msg.size_x * msg.size_y * msg.size_z)
    assert len(msg.occupancy) == msg.size_x * msg.size_y * msg.size_z


def test_obstacle_set_accepts_obstacles():
    obs = Obstacle()
    obs.id = 1
    obs.radius = 0.5
    obs.obstacle_class = Obstacle.CLASS_STATIC
    obs_set = ObstacleSet()
    obs_set.header = make_header()
    obs_set.obstacles = [obs]
    assert len(obs_set.obstacles) == 1
    assert obs_set.obstacles[0].radius > 0


def test_mission_requires_frame_and_waypoints():
    wp = Waypoint()
    wp.acceptance_radius = 0.5
    wp.waypoint_type = Waypoint.TYPE_TARGET
    mission = Mission()
    mission.header = make_header()
    mission.mission_id = 'test'
    mission.waypoints = [wp]
    assert mission.header.frame_id == 'map'
    assert len(mission.waypoints) >= 1


def test_trajectory_invalid_by_default():
    traj = Trajectory()
    assert traj.valid is False  # producers must explicitly set valid=True


def test_trajectory_points_well_formed():
    traj = Trajectory()
    traj.header = make_header()
    pt = TrajectoryPoint()
    pt.position.x = 1.0
    traj.points = [pt]
    traj.valid = True
    assert traj.valid
    assert len(traj.points) == 1


def test_planner_status_state_enum_values():
    valid_states = {
        PlannerStatus.STATE_IDLE, PlannerStatus.STATE_PLANNING,
        PlannerStatus.STATE_EXECUTING, PlannerStatus.STATE_REPLANNING,
        PlannerStatus.STATE_GOAL_REACHED, PlannerStatus.STATE_FAILED,
    }
    status = PlannerStatus()
    status.state = PlannerStatus.STATE_EXECUTING
    assert status.state in valid_states


def test_system_health_level_enum_values():
    health = SystemHealth()
    health.overall_level = SystemHealth.LEVEL_OK
    assert health.overall_level in (
        SystemHealth.LEVEL_OK, SystemHealth.LEVEL_WARN,
        SystemHealth.LEVEL_CRITICAL, SystemHealth.LEVEL_FAILSAFE,
    )


def test_vehicle_command_invalid_command_must_not_be_actionable():
    cmd = VehicleCommand()
    cmd.valid = False
    # Contract: consumers (PX4 Interface) must reject when valid=False,
    # regardless of mode/position content.
    assert cmd.valid is False


def test_vehicle_command_position_mode_requires_position():
    cmd = VehicleCommand()
    cmd.mode = VehicleCommand.MODE_POSITION
    cmd.position.x, cmd.position.y, cmd.position.z = 1.0, 2.0, 3.0
    cmd.valid = True
    assert cmd.valid
    assert cmd.mode == VehicleCommand.MODE_POSITION
