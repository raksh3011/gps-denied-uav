"""MockMission: publishes a fixed, latched Mission on startup and reports
PlannerStatus, so the rest of the pipeline can be exercised without a real
mission file loader or operator input UI.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from uav_interfaces.msg import Mission, Waypoint, PlannerStatus


class MockMission(Node):
    def __init__(self):
        super().__init__('mock_mission')
        mission_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.mission_pub = self.create_publisher(Mission, '/mission/current', mission_qos)
        self.create_subscription(PlannerStatus, '/planning/status', self.on_status, sensor_qos)

        self.publish_mission()
        self.timer = self.create_timer(1.0, self.publish_mission)

    def publish_mission(self):
        m = Mission()
        m.header.stamp = self.get_clock().now().to_msg()
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
        self.mission_pub.publish(m)

    def on_status(self, msg: PlannerStatus):
        self.get_logger().debug(f'planner state={msg.state} progress={msg.progress}')


def main(args=None):
    rclpy.init(args=args)
    node = MockMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
