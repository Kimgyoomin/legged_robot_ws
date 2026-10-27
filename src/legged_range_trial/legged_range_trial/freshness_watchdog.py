"""Forward Collision Monitor output without the final swept-costmap guard.

The inherited steady-clock timer still stops on missing/stale cloud, odometry,
costmap or safe command, nonfinite commands, and an unready obstacle filter.
Keep map/pose validity checks here, but do not regulate speed or test obstacle
cells again. This experiment relies on the Nav2 controller's collision checks.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import TransformException

from legged_bringup.command_watchdog import CommandWatchdog
from legged_bringup.costmap_guard import planar_yaw


class FreshnessWatchdog(CommandWatchdog):
    def __init__(self):
        super().__init__()
        self.get_logger().info(
            'Trial: final costmap collision guard OFF; Collision Monitor output '
            'is forwarded unchanged when data and pose checks pass.')

    def check_motion(self):
        self.output_command = Twist()
        if self.guard.grid is None:
            return 'invalid_costmap'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.guard.frame, self.base_frame, Time())
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            # Same static-chain convention as the original watchdog.
            if stamp_ns and not -.1 <= age <= self.pose_timeout:
                return 'costmap_pose_stale'
            position = transform.transform.translation
            yaw = planar_yaw(transform.transform.rotation)
            if not all(math.isfinite(v) for v in (position.x, position.y, yaw)):
                return 'invalid_guard_input'
        except (TransformException, ValueError):
            return 'costmap_pose_unavailable'
        # Only /legged/cmd_safe reaches here: upstream stop/rotation commands
        # must remain exactly as Collision Monitor published them.
        self.output_command = self.command
        self.command_scale = 1.0
        return 'pass'


def main():
    rclpy.init()
    node = FreshnessWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
