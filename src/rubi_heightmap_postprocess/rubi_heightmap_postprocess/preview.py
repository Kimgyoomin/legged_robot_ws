"""Publish immutable comparison clouds on separate topics at original map time."""
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class Preview(Node):
    def __init__(self):
        super().__init__('rubi_heightmap_postprocess_preview')
        result = Path(self.declare_parameter('result_dir', '').value)
        if not (result/'metrics.json').is_file():
            raise ValueError('result_dir must contain a completed comparison (metrics.json)')
        data = np.load(result/'layers.npz', allow_pickle=False)
        frame = str(data['frame']); stamp = int(data['stamp_ns'])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.clouds=[]
        for name in ('raw', 'baseline', 'candidate'):
            valid=np.isfinite(data[name])
            xyz=np.column_stack([data['x'][valid],data['y'][valid],data[name][valid]]).astype('<f4')
            msg=PointCloud2();msg.header.frame_id=frame
            msg.header.stamp.sec=stamp//1_000_000_000;msg.header.stamp.nanosec=stamp%1_000_000_000
            msg.height=1;msg.width=len(xyz);msg.is_dense=True;msg.is_bigendian=False
            msg.point_step=12;msg.row_step=len(xyz)*12
            msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
            msg.data=xyz.tobytes()
            pub=self.create_publisher(PointCloud2,'/rubi/postprocess_preview/'+name,qos)
            self.clouds.append((pub,msg))
        data.close()
        self.publish_all();self.timer=self.create_timer(2.,self.publish_all)
        self.get_logger().info(f'Frozen comparison: {result}; frame={frame}, original stamp={stamp}; no live map output')

    def publish_all(self):
        for pub,msg in self.clouds:
            pub.publish(msg)


def main():
    rclpy.init();node=None
    try:
        node=Preview();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
