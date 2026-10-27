"""Transport FAST-LIO's measured twist to base_link, without publishing TF."""
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_multiply

from legged_range_trial.controller_odom import rotation
from legged_range_trial.twist_frame import transport


class FastlioControllerOdom(Node):
    def __init__(self):
        super().__init__('fastlio_controller_odom')
        param = lambda key, default: self.declare_parameter(key, default).value
        source = param('input_topic', '/fastlio/odometry_velocity')
        target = param('output_topic', '/legged/controller_odom')
        if source == target:
            raise ValueError('Input and output topics must differ')
        self.base = param('base_frame', 'base_link')
        self.max_age = param('max_age', .5)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Odometry, target, 10)
        self.sub = self.create_subscription(Odometry, source, self.receive, qos_profile_sensor_data)
        self.get_logger().info(f'FAST-LIO state velocity: {source} -> {target}, '
                               f'origin/axes={self.base}; no pose differentiation or TF output')

    def receive(self, msg):
        if not msg.header.frame_id or not msg.child_frame_id:
            return
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        if age > self.max_age or age < -.05:
            self.get_logger().warning('Dropping stale/future FAST-LIO velocity', throttle_duration_sec=5.)
            return
        try:
            mount_rot = np.eye(3)
            offset = np.zeros(3)
            mount_quat = np.array([0., 0., 0., 1.])
            if msg.child_frame_id != self.base:
                mount = self.tf.lookup_transform(msg.child_frame_id, self.base,
                    Time.from_msg(msg.header.stamp)).transform
                mount_rot = rotation(mount.rotation)
                offset = np.array([mount.translation.x, mount.translation.y, mount.translation.z])
                mount_quat = np.array([mount.rotation.x, mount.rotation.y,
                                       mount.rotation.z, mount.rotation.w])
            linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
            twist = np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])
            covariance = np.array(msg.twist.covariance).reshape(6, 6)
            twist, covariance = transport(twist, covariance, mount_rot, offset)
            pose = msg.pose.pose
            pose_rot = rotation(pose.orientation)
            position = np.array([pose.position.x, pose.position.y, pose.position.z]) + pose_rot @ offset
            q = pose.orientation
            orientation = quaternion_multiply([q.x, q.y, q.z, q.w], mount_quat)
            if not all(np.isfinite(v).all() for v in (twist, covariance, position, orientation)):
                raise ValueError('Nonfinite FAST-LIO velocity or transform')
        except (TransformException, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            return
        out = copy.deepcopy(msg)
        out.child_frame_id = self.base
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, position)
        q = out.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, orientation)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, twist[:3])
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, twist[3:])
        out.twist.covariance = covariance.ravel().tolist()
        # This stream supplies controller twist only; its transformed pose
        # covariance is not intended for estimator fusion.
        out.pose.covariance = [1e6 if i % 7 == 0 else 0. for i in range(36)]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FastlioControllerOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
