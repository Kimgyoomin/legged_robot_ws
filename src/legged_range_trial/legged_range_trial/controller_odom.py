"""Estimate measured planar twist from FAST-LIO pose, without publishing TF.

The output is for Nav2 control only. Full-pose covariance is deliberately large:
this derived stream must not be fed back into localization or sensor fusion.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

from legged_range_trial.pose_velocity import PoseVelocity


def rotation(q):
    v = np.array([q.x,q.y,q.z,q.w],dtype=float)
    norm = np.linalg.norm(v)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x,y,z,w = v/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class ControllerOdom(Node):
    def __init__(self):
        super().__init__('controller_odom')
        param = lambda k,v: self.declare_parameter(k,v).value
        source = param('input_topic','/Odometry')
        target = param('output_topic','/legged/controller_odom')
        if source == target:
            raise ValueError('Input and output topics must differ')
        self.base = param('base_frame','base_link')
        self.fit = PoseVelocity(window=param('window',.4), min_span=param('min_span',.2),
            max_gap=param('max_gap',.5), max_speed=param('max_speed',3.),
            max_yaw_rate=param('max_yaw_rate',4.))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(Odometry,target,10)
        self.sub = self.create_subscription(Odometry,source,self.receive,qos_profile_sensor_data)
        self.frames = None
        self.get_logger().info(f'Controller velocity: {source} pose -> {target}; '
                               f'{self.fit.window:.2f}s fit, base={self.base}; no TF output')

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not msg.header.frame_id or not msg.child_frame_id:
            self.fit.reset('missing_frame')
            return
        frames = (msg.header.frame_id,msg.child_frame_id)
        if frames != self.frames:
            self.fit.reset('frame_change')
            self.frames = frames
        try:
            pose = msg.pose.pose
            rot = rotation(pose.orientation)
            position = np.array([pose.position.x,pose.position.y,pose.position.z])
            # Lookup at the sample timestamp: do not use a newer robot pose.
            if msg.child_frame_id != self.base:
                tf = self.tf.lookup_transform(msg.child_frame_id,self.base,
                                               Time.from_msg(msg.header.stamp)).transform
                offset = np.array([tf.translation.x,tf.translation.y,tf.translation.z])
                position += rot @ offset
                rot = rot @ rotation(tf.rotation)
            if not np.isfinite(position).all():
                raise ValueError('Nonfinite position')
            yaw = math.atan2(rot[1,0],rot[0,0])
            velocity = self.fit.add(stamp,float(position[0]),float(position[1]),yaw)
        except (TransformException,ValueError) as error:
            self.fit.reset('pose_or_mount_unavailable')
            self.get_logger().warning(str(error),throttle_duration_sec=5.)
            return
        if velocity is None:
            return
        out = Odometry()
        out.header = msg.header
        # Planar projection at the base origin, with yaw-aligned twist axes.
        out.child_frame_id = self.base
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z = map(float,position)
        out.pose.pose.orientation.z = math.sin(yaw/2)
        out.pose.pose.orientation.w = math.cos(yaw/2)
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.angular.z = velocity
        for axis in range(6):
            out.pose.covariance[axis*7] = 1e6
            out.twist.covariance[axis*7] = 1e6
        # Conservative fit uncertainty, not calibrated sensor covariance.
        out.twist.covariance[0] = out.twist.covariance[7] = .04
        out.twist.covariance[35] = .04
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ControllerOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
