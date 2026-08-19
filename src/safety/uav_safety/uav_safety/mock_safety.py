"""MockSafety: the single gate between Planning and the vehicle interface.
Validates incoming Trajectory + LocalizationState, forwards a VehicleCommand
only when both are healthy, and publishes SystemHealth. Real watchdogs and
failure/recovery state machine come later; this proves the contract.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from uav_interfaces.msg import (
    Trajectory, LocalizationState, VehicleCommand, SystemHealth,
)


class MockSafety(Node):
    def __init__(self):
        super().__init__('mock_safety')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(Trajectory, '/planning/trajectory', self.on_traj, qos)
        self.create_subscription(LocalizationState, '/localization/state', self.on_loc, qos)

        self.cmd_pub = self.create_publisher(VehicleCommand, '/safety/vehicle_command', qos)
        self.health_pub = self.create_publisher(SystemHealth, '/safety/system_health', qos)

        self.last_traj = None
        self.last_traj_time = 0.0
        self.last_loc = None
        self.last_loc_time = 0.0

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('staleness_timeout_s', 0.5)
        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_traj(self, msg: Trajectory):
        self.last_traj = msg
        self.last_traj_time = time.monotonic()

    def on_loc(self, msg: LocalizationState):
        self.last_loc = msg
        self.last_loc_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        timeout = self.get_parameter('staleness_timeout_s').value
        stamp = self.get_clock().now().to_msg()

        health = SystemHealth()
        health.header.stamp = stamp

        loc_stale = self.last_loc is None or (now - self.last_loc_time) > timeout
        traj_stale = self.last_traj is None or (now - self.last_traj_time) > timeout
        loc_ok = self.last_loc is not None and self.last_loc.localization_ok
        traj_ok = self.last_traj is not None and self.last_traj.valid

        cmd = VehicleCommand()
        cmd.header.stamp = stamp

        if loc_stale or traj_stale or not loc_ok or not traj_ok:
            health.overall_level = SystemHealth.LEVEL_CRITICAL
            faults = []
            if loc_stale:
                faults.append('LOC_STALE')
            if traj_stale:
                faults.append('TRAJ_STALE')
            if self.last_loc is not None and not loc_ok:
                faults.append('LOC_NOT_OK')
            if self.last_traj is not None and not traj_ok:
                faults.append('TRAJ_INVALID')
            health.active_faults = faults
            cmd.mode = VehicleCommand.MODE_HOLD
            cmd.valid = False
        else:
            health.overall_level = SystemHealth.LEVEL_OK
            target = self.last_traj.points[-1]
            cmd.mode = VehicleCommand.MODE_POSITION
            cmd.position = target.position
            cmd.yaw = target.yaw
            cmd.valid = True

        self.cmd_pub.publish(cmd)
        self.health_pub.publish(health)


def main(args=None):
    rclpy.init(args=args)
    node = MockSafety()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
