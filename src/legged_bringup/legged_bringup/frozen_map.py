"""CPU-only frozen elevation publisher. Acquisition stamps remain unchanged."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from .map_io import read_snapshot


class FrozenMap(Node):
    def __init__(self):
        super().__init__('frozen_map')
        filename = self.declare_parameter('file', '').value
        frame = self.declare_parameter('frame', 'map').value
        snap = read_snapshot(filename)
        if snap.frame != frame:
            raise ValueError('Frozen map frame differs; transform and regrid offline first')
        x, y = snap.xy()
        valid = np.isfinite(snap.grid)
        xyz = np.column_stack([x[valid], y[valid], snap.grid[valid]]).astype('<f4')
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = frame
        self.cloud.header.stamp.sec = snap.stamp_ns // 1_000_000_000
        self.cloud.header.stamp.nanosec = snap.stamp_ns % 1_000_000_000
        self.cloud.height = 1
        self.cloud.width = len(xyz)
        self.cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                             for i,n in enumerate(('x','y','z'))]
        self.cloud.point_step = 12
        self.cloud.row_step = len(xyz)*12
        self.cloud.is_dense = True
        self.cloud.data = xyz.tobytes()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/legged/height', qos)
        info = self.create_publisher(String, '/legged/height_info', qos)
        self.info = info
        info.publish(String(data=json.dumps({'kind':'frozen', 'sha256':snap.digest,
            'frame':frame,'resolution':snap.resolution,'acquisition_stamp_ns':snap.stamp_ns,
            'cells':len(xyz),'file':str(snap.source)})))
        self.create_timer(1.0, lambda:self.publisher.publish(self.cloud))
        self.publisher.publish(self.cloud)
        self.get_logger().info(f'Frozen height loaded: {len(xyz)} cells; {snap.digest}')


def main():
    rclpy.init()
    node = FrozenMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
