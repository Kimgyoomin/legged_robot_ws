"""Last command stage: zero output if the upstream controller/monitor goes away."""
import math
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from legged_bringup.costmap_guard import CostmapGuard, planar_yaw


class CommandWatchdog(Node):
    def __init__(self):
        super().__init__('command_watchdog')
        self.timeout = self.declare_parameter('timeout', 0.3).value
        self.last = 0.0
        self.command = Twist()
        self.last_odom = self.last_costmap = self.last_cloud = 0.0
        self.odom_stamp = self.costmap_stamp = self.cloud_stamp = None
        self.cloud_timeout = self.declare_parameter("cloud_timeout", .4).value
        self.create_subscription(PointCloud2, "/legged/collision_points", self.receive_cloud, qos_profile_sensor_data)
        self.collision_input_state = 'waiting_for_scan_and_mount'
        self.tracking_state = 'unknown'
        self.raw = Twist()
        self.last_raw = 0.
        self.last_reason = None
        self.last_status = 0.
        self.create_subscription(String, '/legged/collision_input_state', self.receive_input_state, 1)
        self.create_subscription(String, '/legged/tracking_state', self.receive_tracking_state, 1)
        self.create_subscription(Twist, '/legged/cmd_raw', self.receive_raw, 1)
        self.status_pub = self.create_publisher(String, '/legged/motion_status', 1)
        self.source_timeout = self.declare_parameter('source_timeout', 1.0).value
        odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        self.create_subscription(Odometry, odom_topic, self.receive_odom, 1)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', self.receive_costmap, 1)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.pose_timeout = self.declare_parameter('max_pose_age', .5).value
        self.guard = CostmapGuard(
            half_length=self.declare_parameter('footprint_half_length', .5).value,
            half_width=self.declare_parameter('footprint_half_width', .45).value,
            horizon=self.declare_parameter('prediction_horizon', 1.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Twist, '/legged/cmd_safe', self.receive, 1)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        values = [v for a in (msg.linear,msg.angular) for v in (a.x,a.y,a.z)]
        if all(math.isfinite(v) for v in values):
            self.command = msg
            self.last = time.monotonic()
        else:
            self.last = 0.0

    def receive_input_state(self, msg):
        self.collision_input_state = msg.data

    def receive_tracking_state(self, msg):
        self.tracking_state = msg.data

    def receive_raw(self, msg):
        self.raw = msg
        self.last_raw = time.monotonic()

    def receive_cloud(self, msg):
        self.last_cloud = time.monotonic()
        self.cloud_stamp = msg.header.stamp

    def receive_odom(self, msg):
        self.last_odom = time.monotonic()
        self.odom_stamp = msg.header.stamp

    def receive_costmap(self, msg):
        self.last_costmap = time.monotonic()
        self.costmap_stamp = msg.header.stamp
        self.guard.update(msg)

    def check_motion(self):
        if self.guard.grid is None:
            return 'invalid_costmap'
        try:
            t = self.tf_buffer.lookup_transform(self.guard.frame, self.base_frame, Time())
            stamp_ns = t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
            # A zero stamp is valid for an entirely static test transform chain.
            if stamp_ns and not -.1 <= age <= self.pose_timeout:
                return 'costmap_pose_stale'
            p = t.transform.translation
            pose = (p.x, p.y, planar_yaw(t.transform.rotation))
        except (TransformException, ValueError):
            return 'costmap_pose_unavailable'
        return self.guard.check(pose, (self.command.linear.x, self.command.linear.y,
                                      self.command.angular.z))

    def tick(self):
        steady = time.monotonic()
        now = self.get_clock().now().nanoseconds
        def fresh(received, stamp, timeout):
            if stamp is None or steady-received > timeout: return False
            age = (now-stamp.sec*1_000_000_000-stamp.nanosec)/1e9
            return -.1 <= age <= timeout
        if self.collision_input_state != 'ready': reason = self.collision_input_state
        elif not fresh(self.last_cloud,self.cloud_stamp,self.cloud_timeout): reason = 'collision_cloud_timeout'
        elif not fresh(self.last_odom,self.odom_stamp,self.source_timeout): reason = 'odometry_timeout'
        elif not fresh(self.last_costmap,self.costmap_stamp,self.source_timeout): reason = 'costmap_timeout'
        elif steady-self.last >= self.timeout: reason = 'safe_command_timeout'
        else: reason = self.check_motion()
        ready = reason == 'pass'
        output = self.command if ready else Twist()
        self.pub.publish(output)
        if ready and abs(output.linear.x) < 1.e-6 and abs(output.angular.z) < 1.e-6:
            raw_moving = abs(self.raw.linear.x) > 1.e-6 or abs(self.raw.angular.z) > 1.e-6
            reason = 'collision_or_upstream_stop' if raw_moving and steady-self.last_raw < self.timeout else self.tracking_state
        if reason != self.last_reason or steady-self.last_status > .5:
            self.status_pub.publish(String(data=json.dumps({
                'gate': 'open' if ready else 'stopped', 'reason': reason,
                'tracking': self.tracking_state, 'collision_input': self.collision_input_state,
                'vx': output.linear.x, 'wz': output.angular.z})))
            if reason != self.last_reason:
                self.get_logger().info('Motion status: ' + reason)
            self.last_reason, self.last_status = reason, steady


def main():
    rclpy.init()
    node = CommandWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
