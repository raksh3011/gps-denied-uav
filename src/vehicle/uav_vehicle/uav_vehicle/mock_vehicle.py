"""MockVehicle: consumes VehicleCommand from Safety and logs what would be
sent to PX4. Rejects any command where `valid == False`. This proves the
Safety -> PX4 Interface contract before the real MAVROS/uXRCE-DDS bridge and
Gazebo/PX4 SITL integration exist.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import VehicleCommand


class MockVehicle(Node):
    def __init__(self):
        super().__init__('mock_vehicle')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommand, '/safety/vehicle_command', self.on_cmd, qos)
        self.accepted = 0
        self.rejected = 0

    def on_cmd(self, msg: VehicleCommand):
        if not msg.valid:
            self.rejected += 1
            self.get_logger().warn(
                f'Rejected invalid VehicleCommand (mode={msg.mode}), holding last safe state.')
            return
        self.accepted += 1
        self.get_logger().info(
            f'Would forward to PX4: mode={msg.mode} '
            f'pos=({msg.position.x:.2f},{msg.position.y:.2f},{msg.position.z:.2f}) '
            f'yaw={msg.yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MockVehicle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
