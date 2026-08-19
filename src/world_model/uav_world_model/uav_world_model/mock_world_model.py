"""MockWorldModel: subscribes to LocalizationState and publishes a fixed
LocalMap + ObstacleSet so Planning/Safety can be developed independently of
real LiDAR preprocessing and mapping.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import LocalizationState, LocalMap, ObstacleSet, Obstacle


class MockWorldModel(Node):
    def __init__(self):
        super().__init__('mock_world_model')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            LocalizationState, '/localization/state', self.on_localization, qos)
        self.map_pub = self.create_publisher(LocalMap, '/world_model/local_map', qos)
        self.obstacle_pub = self.create_publisher(ObstacleSet, '/world_model/obstacles', qos)

        self.declare_parameter('rate_hz', 5.0)
        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.last_pose = None

    def on_localization(self, msg: LocalizationState):
        self.last_pose = msg.pose

    def tick(self):
        now = self.get_clock().now().to_msg()

        m = LocalMap()
        m.header.stamp = now
        m.header.frame_id = 'map'
        m.resolution = 0.2
        m.size_x = m.size_y = m.size_z = 50
        if self.last_pose is not None:
            m.origin.x = self.last_pose.position.x - 5.0
            m.origin.y = self.last_pose.position.y - 5.0
            m.origin.z = 0.0
        m.occupancy = [0] * (m.size_x * m.size_y * m.size_z)
        m.map_valid = True
        self.map_pub.publish(m)

        obs_set = ObstacleSet()
        obs_set.header.stamp = now
        obs_set.header.frame_id = 'map'
        obstacle = Obstacle()
        obstacle.id = 1
        obstacle.position.x = 3.0
        obstacle.position.y = 0.0
        obstacle.position.z = 2.0
        obstacle.radius = 0.5
        obstacle.obstacle_class = Obstacle.CLASS_STATIC
        obs_set.obstacles = [obstacle]
        self.obstacle_pub.publish(obs_set)


def main(args=None):
    rclpy.init(args=args)
    node = MockWorldModel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
