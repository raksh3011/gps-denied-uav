"""MockLocalization: publishes a plausible LocalizationState at a fixed rate
so downstream modules (World Model, Planning, Safety) can be developed and
tested without a real LiDAR-Inertial Odometry pipeline.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import LocalizationState


class MockLocalization(Node):
    def __init__(self):
        super().__init__('mock_localization')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(LocalizationState, '/localization/state', qos)
        self.declare_parameter('rate_hz', 50.0)
        rate = self.get_parameter('rate_hz').value
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        msg = LocalizationState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'

        # Gentle circular trajectory as a stand-in for real odometry.
        radius = 5.0
        omega = 0.1
        msg.pose.position.x = radius * math.cos(omega * t)
        msg.pose.position.y = radius * math.sin(omega * t)
        msg.pose.position.z = 2.0
        msg.pose.orientation.w = 1.0

        msg.twist.linear.x = -radius * omega * math.sin(omega * t)
        msg.twist.linear.y = radius * omega * math.cos(omega * t)

        msg.pose_covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        msg.twist_covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        msg.confidence = 0.95
        msg.localization_ok = True
        msg.status = LocalizationState.STATUS_NOMINAL

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
