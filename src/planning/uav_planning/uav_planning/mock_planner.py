"""MockPlanner: consumes Mission + LocalizationState + LocalMap/ObstacleSet
and publishes a trivial straight-line Trajectory plus PlannerStatus, so
Safety/Mission/PX4-Interface can be developed independently of real
global/local planning algorithms.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration

from uav_interfaces.msg import (
    Mission, LocalizationState, LocalMap, ObstacleSet,
    Trajectory, TrajectoryPoint, PlannerStatus,
)


class MockPlanner(Node):
    def __init__(self):
        super().__init__('mock_planner')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        mission_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Mission, '/mission/current', self.on_mission, mission_qos)
        self.create_subscription(LocalizationState, '/localization/state', self.on_loc, sensor_qos)
        self.create_subscription(LocalMap, '/world_model/local_map', self.on_map, sensor_qos)
        self.create_subscription(ObstacleSet, '/world_model/obstacles', self.on_obstacles, sensor_qos)

        self.traj_pub = self.create_publisher(Trajectory, '/planning/trajectory', sensor_qos)
        self.status_pub = self.create_publisher(PlannerStatus, '/planning/status', sensor_qos)

        self.mission = None
        self.loc = None
        self.have_map = False
        self.have_obstacles = False

        self.declare_parameter('rate_hz', 10.0)
        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_mission(self, msg: Mission):
        self.mission = msg

    def on_loc(self, msg: LocalizationState):
        self.loc = msg

    def on_map(self, msg: LocalMap):
        self.have_map = msg.map_valid

    def on_obstacles(self, msg: ObstacleSet):
        self.have_obstacles = True

    def tick(self):
        status = PlannerStatus()
        status.header.stamp = self.get_clock().now().to_msg()

        if self.mission is None or self.loc is None or not self.have_map:
            status.state = PlannerStatus.STATE_IDLE
            status.message = 'waiting for mission/localization/map'
            self.status_pub.publish(status)
            return

        if not self.loc.localization_ok or not self.mission.waypoints:
            status.state = PlannerStatus.STATE_FAILED
            status.message = 'invalid localization or empty mission'
            self.status_pub.publish(status)
            return

        target = self.mission.waypoints[0].position

        traj = Trajectory()
        traj.header.stamp = status.header.stamp
        traj.header.frame_id = 'map'
        n_points = 10
        p0 = self.loc.pose.position
        for i in range(n_points + 1):
            frac = i / n_points
            pt = TrajectoryPoint()
            pt.time_from_start = Duration(sec=int(frac * 5), nanosec=0)
            pt.position.x = p0.x + frac * (target.x - p0.x)
            pt.position.y = p0.y + frac * (target.y - p0.y)
            pt.position.z = p0.z + frac * (target.z - p0.z)
            traj.points.append(pt)
        traj.valid = True
        self.traj_pub.publish(traj)

        status.state = PlannerStatus.STATE_EXECUTING
        status.message = ''
        status.progress = 0.0
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
