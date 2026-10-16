# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
# RUBI LiDAR-only ROS adapter. Original stamped-TF queue controller retained.
from __future__ import annotations
from dataclasses import fields
import math
import os
import time as monotonic_time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
import tf2_ros
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from .core import ElevationMap
from .parameter import Parameter
from .stamped_tf_queue import PendingPointCloud, PendingPointCloudQueue
from .gridmap_utils import encode_layer_to_multiarray
from .pointcloud import pointcloud_xyz, quaternion_matrix
from .odometry import OdometryBuffer, compose_pose

class HeightMappingNode(Node):
    def __init__(self):
        super().__init__('rubi_height_mapping')
        def setting(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        self.param = Parameter()
        for field in fields(self.param):
            if field.name != 'weights_path':
                setattr(self.param, field.name, setting(field.name, getattr(self.param, field.name)))
        self.param.weights_path = os.path.join(get_package_share_directory('rubi_height_mapping'),
                                              'config', 'traversability_weights.npz')
        self.param.update()
        self.map_frame = setting('map_frame', 'map')
        self.base_frame = setting('base_frame', 'base_link')
        self.odom_topic = setting('odom_topic', '')
        odom_cache = setting('odom_cache_seconds', 10.0)
        odom_gap = setting('odom_max_gap', 0.2)
        if not isinstance(self.odom_topic, str):
            raise ValueError('odom_topic must be a string; empty selects TF mode')
        self.odom_topic = self.odom_topic.strip()
        self._odom = OdometryBuffer(self.map_frame, odom_cache, odom_gap)
        self._odom_received = self._odom_rejected = 0
        self.input_topic = setting('input_topic', '/livox/lidar_PointCloud2')
        output = setting('output_topic', '/rubi/local_elevation/grid_map')
        self.min_range = setting('min_range', 0.6)
        self.max_range = setting('max_range', 3.0)
        self.tf_wait_timeout = setting('tf_wait_timeout', 1.5)
        self.tf_queue_size = setting('tf_queue_size', 20)
        self.tf_queue_max_bytes = setting('tf_queue_max_bytes', 67108864)
        self.tf_retry_period = setting('tf_retry_period', 0.02)
        self.tf_max_scans_per_tick = setting('tf_max_scans_per_tick', 1)
        publish_fps = setting('publish_fps', 5.0)
        variance_fps = setting('update_variance_fps', 5.0)
        statistics_fps = setting('statistics_fps', 0.2)
        if not all(isinstance(v, str) and v.strip() for v in (self.map_frame, self.base_frame, self.input_topic, output)):
            raise ValueError('Frames and topics must be nonempty')
        if not all(math.isfinite(v) for v in (self.min_range, self.max_range, self.tf_wait_timeout, self.tf_retry_period)):
            raise ValueError('Nonfinite range/TF parameters')
        if not 0 <= self.min_range < self.max_range:
            raise ValueError('Require 0 <= min_range < max_range')
        if not all(math.isfinite(v) and 0 < v <= 100 for v in (publish_fps, variance_fps, statistics_fps)):
            raise ValueError('Timer frequencies must be in (0, 100]')
        if self.param.time_interval <= 0:
            raise ValueError('time_interval must be positive')
        self.stamped_tf_queue_enabled = True
        self.allow_latest_tf_fallback = False
        self.my_subscribers = {'lidar': {'topic_name': self.input_topic, 'data_type': 'pointcloud'}}
        self.my_publishers = {'raw': {'layers': ['elevation', 'variance'], 'basic_layers': ['elevation']}}
        self._last_t = self._last_fused_t = None
        self._map_t = self._map_q = None
        self._pointcloud_process_counter = 0
        self._tf_buffer = tf2_ros.Buffer()
        if self.odom_topic:
            # Only fixed mounting transforms are accepted in direct odometry
            # mode. Dynamic localization TF cannot override the selected pose.
            self._static_sub = self.create_subscription(
                TFMessage, '/tf_static', self._receive_static_tf,
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self._odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._receive_odom, qos_profile_sensor_data)
        else:
            self._listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._initialize_input_state()
        self._map = ElevationMap(self.param)
        self._map_data = np.zeros((2, self._map.cell_n-2, self._map.cell_n-2), dtype=np.float32)
        self._publishers_dict = {'raw': self.create_publisher(GridMap, output, 1)}
        self.create_subscription(PointCloud2, self.input_topic, self._receive_cloud, qos_profile_sensor_data)
        self.create_timer(self.tf_retry_period, self.process_pending_pointclouds, clock=self._steady_clock)
        self.create_timer(1.0/publish_fps, lambda: self.publish_map('raw'))
        self.create_timer(1.0/variance_fps, self.update_variance)
        self.create_timer(self.param.time_interval, self.update_time)
        self.create_timer(1.0/statistics_fps, self.publish_input_statistics)
        self.get_logger().info(f'Visibility cleanup: ray_mode={self.param.ray_mode} voxel_size={self.param.ray_voxel_size or self.param.resolution:.3f} m; all height observations retained.')
        self.get_logger().info('Fusion: immutable scan snapshot -> proposal buffers -> per-cell finalization; accepted endpoints take precedence over cleanup.')
        self.get_logger().info('RUBI height-only mapper: CuPy backend, no PyTorch/image/semantic imports; CPU thread settings inherited.')
        self.get_logger().info(
            f"Pose source: {self.odom_topic or 'TF'}; map_frame='{self.map_frame}', "
            f"base_frame='{self.base_frame}'; direct odometry uses /tf_static only for mounting transforms.")

    def _receive_static_tf(self, msg):
        for transform in msg.transforms:
            self._tf_buffer.set_transform_static(transform, 'rubi_static_mounts')

    def _receive_odom(self, msg):
        if self._epoch_faulted or self._fusion_faulted or not self._observe_ros_clock():
            return
        self._odom_received += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            accepted = self._odom.add(self._stamp_to_ns(msg.header.stamp),
                                      msg.header.frame_id, msg.child_frame_id,
                                      [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            if not accepted:
                self._odom_rejected += 1
        except ValueError as exc:
            self._odom_rejected += 1
            self.get_logger().warning(f'Rejected odometry: {exc}', throttle_duration_sec=5.0)

    def _receive_cloud(self, msg):
        self._input_stats['received'] += 1
        self._last_received_stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if self._observe_ros_clock():
            self._enqueue_pointcloud(msg, 'lidar')

    def _prepare_pointcloud(self, msg, sub_key):
        pts = pointcloud_xyz(msg)
        range2 = np.einsum('ij,ij->i', pts, pts)
        pts = pts[(range2 >= self.min_range**2) & (range2 <= self.max_range**2)]
        return pts, ['x', 'y', 'z']

    def update_variance(self):
        if not self._epoch_faulted and not self._fusion_faulted and self._observe_ros_clock():
            self._map.update_variance()

    def update_time(self):
        if not self._epoch_faulted and not self._fusion_faulted and self._observe_ros_clock():
            self._map.update_time()

    def _initialize_input_state(self) -> None:
        if self.stamped_tf_queue_enabled and self.allow_latest_tf_fallback:
            raise ValueError(
                "stamped_tf_queue_enabled and allow_latest_tf_fallback cannot both be true"
            )
        if self.tf_wait_timeout <= 0.0 or self.tf_retry_period <= 0.0:
            raise ValueError("tf_wait_timeout and tf_retry_period must be positive")
        if self.tf_max_scans_per_tick <= 0:
            raise ValueError("tf_max_scans_per_tick must be positive")

        if self.stamped_tf_queue_enabled:
            pointclouds = [
                (key, cfg) for key, cfg in self.my_subscribers.items()
                if cfg.get('data_type') == 'pointcloud'
            ]
            unsupported = [
                key for key, cfg in self.my_subscribers.items()
                if cfg.get('data_type') != 'pointcloud' or cfg.get('channels')
            ]
            if len(pointclouds) != 1 or unsupported:
                raise ValueError(
                    "stamped TF queue supports exactly one XYZ-only pointcloud subscriber; "
                    f"pointclouds={len(pointclouds)}, unsupported={unsupported}"
                )

        self._pending_clouds = PendingPointCloudQueue(
            self.tf_queue_size, self.tf_queue_max_bytes
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_ros_now_ns = None
        self._epoch_faulted = False
        self._fusion_faulted = False
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None
        self._queue_wait_samples = []
        self._last_tf_errors = {}
        self._last_tf_error_stamp_ns = None
        self._last_received_stamp_ns = None
        self._input_stats = {
            'received': 0,
            'fused': 0,
            'timeout_drops': 0,
            'overflow_drops': 0,
            'invalid_drops': 0,
            'duplicate_out_of_order_drops': 0,
            'reset_drops': 0,
        }

        topics = [
            str(cfg.get('topic_name', '')) for cfg in self.my_subscribers.values()
            if cfg.get('data_type') == 'pointcloud'
        ]
        self.get_logger().info(
            "Input configuration: topics=%s map_frame='%s' base_frame='%s' "
            "stamped_tf_queue_enabled=%s allow_latest_tf_fallback=%s"
            % (topics, self.map_frame, self.base_frame,
               self.stamped_tf_queue_enabled, self.allow_latest_tf_fallback)
        )

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    @staticmethod
    def _stamp_text(stamp_ns: int) -> str:
        return f"{stamp_ns // 1000000000}.{stamp_ns % 1000000000:09d}"

    def _enqueue_pointcloud(self, msg: PointCloud2, sub_key: str) -> None:
        if self._epoch_faulted or self._fusion_faulted:
            self._input_stats['invalid_drops'] += 1
            return
        frame_id = str(msg.header.frame_id).strip()
        payload_bytes = len(msg.data)
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if not frame_id or payload_bytes == 0:
            self._input_stats['invalid_drops'] += 1
            return

        item = PendingPointCloud(
            message=msg,
            subscriber_key=sub_key,
            stamp_ns=stamp_ns,
            received_monotonic=monotonic_time.monotonic(),
            payload_bytes=payload_bytes,
        )
        result, overflowed = self._pending_clouds.push(item)
        if result != 'accepted':
            if result in ('duplicate', 'out_of_order'):
                self._input_stats['duplicate_out_of_order_drops'] += 1
            elif result == 'oversize':
                self._input_stats['overflow_drops'] += 1
            else:
                self._input_stats['invalid_drops'] += 1
            self.get_logger().warning(
                f"Dropping PointCloud2 stamp={self._stamp_text(stamp_ns)}: {result}",
                throttle_duration_sec=5.0,
            )
            return

        if overflowed:
            self._input_stats['overflow_drops'] += len(overflowed)
            overflow_stamps = {d.stamp_ns for d in overflowed}
            if self._prepared_pending_stamp_ns in overflow_stamps:
                self._clear_prepared_pending()
            self.get_logger().warning(
                "Stamped-TF queue overflow; dropped oldest %d scan(s), pending=%d bytes=%d"
                % (len(overflowed), len(self._pending_clouds), self._pending_clouds.payload_bytes),
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _transform_to_numpy(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)
        return trans, rot

    def _lookup_original_stamp(self, source_frame: str, stamp):
        if source_frame == self.map_frame:
            return True, None, None
        try:
            if self.odom_topic:
                pose = self._odom.lookup(self._stamp_to_ns(stamp))
                position, quaternion = pose.position, pose.quaternion
                if source_frame != self._odom.child_frame:
                    mount = self._tf_buffer.lookup_transform(
                        self._odom.child_frame, source_frame, Time())
                    p, q = mount.transform.translation, mount.transform.rotation
                    position, quaternion = compose_pose(
                        position, quaternion, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                transform = TransformStamped()
                transform.header.frame_id = self.map_frame
                transform.header.stamp = stamp
                transform.child_frame_id = source_frame
                transform.transform.translation = Vector3(**dict(zip('xyz', map(float, position))))
                transform.transform.rotation = Quaternion(**dict(zip('xyzw', map(float, quaternion))))
                return True, transform, None
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, source_frame, stamp
            )
            return True, transform, None
        except Exception as exc:
            return False, None, str(exc)

    def _clear_prepared_pending(self) -> None:
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None

    def _discard_pending_head(self) -> PendingPointCloud:
        item = self._pending_clouds.pop()
        if self._prepared_pending_stamp_ns == item.stamp_ns:
            self._clear_prepared_pending()
        return item

    def _observe_ros_clock(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ros_now_ns is not None and now_ns < self._last_ros_now_ns:
            dropped = self._pending_clouds.clear()
            self._clear_prepared_pending()
            self._input_stats['reset_drops'] += dropped
            self._epoch_faulted = True
            self._last_fused_t = None
            self._odom.clear()
            try:
                self._tf_buffer.clear()
            except AttributeError:
                pass
            self.get_logger().fatal(
                "ROS time moved backwards (%d -> %d). Fusion and normal publication are stopped; "
                "restart/reset both rubi_height_mapping and rubi_global_heightmap_wrapper "
                "before accepting the new epoch." % (self._last_ros_now_ns, now_ns)
            )
            return False
        self._last_ros_now_ns = now_ns
        return True

    def process_pending_pointclouds(self) -> None:
        if not self.stamped_tf_queue_enabled:
            return
        if self._epoch_faulted or self._fusion_faulted or not self._observe_ros_clock():
            return

        processed = 0
        while processed < self.tf_max_scans_per_tick:
            item = self._pending_clouds.peek()
            if item is None:
                return
            now_monotonic = monotonic_time.monotonic()
            wait = now_monotonic - item.received_monotonic
            if wait >= self.tf_wait_timeout:
                self._discard_pending_head()
                self._input_stats['timeout_drops'] += 1
                errors = self._last_tf_errors if self._last_tf_error_stamp_ns == item.stamp_ns else {}
                sensor_error = errors.get('sensor', 'not checked')
                base_error = errors.get('base', 'not checked')
                self.get_logger().warning(
                    "Stamped TF timeout target='%s' sensor_source='%s' base_source='%s' "
                    "stamp=%s wait=%.3fs sensor_error=%s base_error=%s"
                    % (self.map_frame, item.message.header.frame_id, self.base_frame,
                       self._stamp_text(item.stamp_ns), wait, sensor_error, base_error),
                    throttle_duration_sec=5.0,
                )
                processed += 1
                continue

            if self._prepared_pending_stamp_ns != item.stamp_ns:
                try:
                    pts, channels = self._prepare_pointcloud(item.message, item.subscriber_key)
                    if pts.size == 0:
                        raise ValueError("cloud has no finite XYZ points")
                except Exception as exc:
                    self._discard_pending_head()
                    self._input_stats['invalid_drops'] += 1
                    self.get_logger().warning(
                        f"Invalid queued PointCloud2 stamp={self._stamp_text(item.stamp_ns)}: {exc}"
                    )
                    processed += 1
                    continue
                self._prepared_pending_stamp_ns = item.stamp_ns
                self._prepared_pending = (pts, channels)

            sensor_ready, sensor_tf, sensor_error = self._lookup_original_stamp(
                item.message.header.frame_id, item.message.header.stamp
            )
            base_ready, base_tf, base_error = self._lookup_original_stamp(
                self.base_frame, item.message.header.stamp
            )
            self._last_tf_errors = {'sensor': sensor_error, 'base': base_error}
            self._last_tf_error_stamp_ns = item.stamp_ns
            if not sensor_ready or not base_ready:
                # Return promptly so this SingleThreadedExecutor can receive the
                # delayed /tf or /tf_static data before the next steady tick.
                return

            pts, channels = self._prepared_pending
            try:
                if base_tf is None:
                    base_trans = np.zeros(3, dtype=np.float32)
                    base_rot = np.eye(3, dtype=np.float32)
                    base_t = Vector3(x=0.0, y=0.0, z=0.0)
                    base_q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                else:
                    base_trans, base_rot = self._transform_to_numpy(base_tf)
                    base_t = base_tf.transform.translation
                    base_q = base_tf.transform.rotation
                if sensor_tf is None:
                    sensor_trans = np.zeros(3, dtype=np.float32)
                    sensor_rot = np.eye(3, dtype=np.float32)
                else:
                    sensor_trans, sensor_rot = self._transform_to_numpy(sensor_tf)

                # These calls must remain ordered and use the two transforms
                # captured above for this exact sensor stamp.
                self._map.move_to(base_trans, base_rot)
                self._map.input_pointcloud(pts, channels, sensor_rot, sensor_trans, 0, 0, points_are_finite=True)
            except Exception as exc:
                # move_to/input_pointcloud mutate GPU state without rollback.
                # Never publish that possibly partial state as a normal result.
                dropped = self._pending_clouds.clear()
                self._clear_prepared_pending()
                self._fusion_faulted = True
                self._last_fused_t = None
                self.get_logger().fatal(
                    "GPU fusion submission failed at stamp=%s; stopped fusion/publication "
                    "because rollback is unavailable (discarded pending=%d): %s"
                    % (self._stamp_text(item.stamp_ns), dropped, exc)
                )
                return

            self._discard_pending_head()
            self._map_t = base_t
            self._map_q = base_q
            self._last_t = item.message.header.stamp
            self._last_fused_t = item.message.header.stamp
            self._pointcloud_process_counter += 1
            self._input_stats['fused'] += 1
            self._queue_wait_samples.append(wait)
            if len(self._queue_wait_samples) > 2048:
                del self._queue_wait_samples[:-2048]
            self._last_tf_errors = {}
            self._last_tf_error_stamp_ns = None
            processed += 1

    def publish_input_statistics(self) -> None:
        if self.odom_topic:
            self.get_logger().info('odom_stats topic=%s received=%d rejected=%d cached=%d child=%s'
                % (self.odom_topic, self._odom_received, self._odom_rejected,
                   len(self._odom.stamps), self._odom.child_frame))
        samples = self._queue_wait_samples
        if samples:
            p50, p95 = np.percentile(np.asarray(samples), [50, 95])
            maximum = max(samples)
        else:
            p50 = p95 = maximum = 0.0
        received_ns = self._last_received_stamp_ns
        fused_ns = None if self._last_fused_t is None else self._stamp_to_ns(self._last_fused_t)
        self.get_logger().info(
            "input_stats received=%d fused=%d pending=%d pending_bytes=%d "
            "timeout_drops=%d overflow_drops=%d invalid_drops=%d "
            "duplicate_out_of_order_drops=%d reset_drops=%d "
            "queue_wait_p50=%.3fs p95=%.3fs max=%.3fs last_received_stamp=%s "
            "last_fused_stamp=%s epoch_faulted=%s fusion_faulted=%s"
            % (
                self._input_stats['received'], self._input_stats['fused'],
                len(self._pending_clouds), self._pending_clouds.payload_bytes,
                self._input_stats['timeout_drops'], self._input_stats['overflow_drops'],
                self._input_stats['invalid_drops'],
                self._input_stats['duplicate_out_of_order_drops'],
                self._input_stats['reset_drops'], p50, p95, maximum,
                'none' if received_ns is None else self._stamp_text(received_ns),
                'none' if fused_ns is None else self._stamp_text(fused_ns),
                self._epoch_faulted, self._fusion_faulted,
            )
        )

    def publish_map(self, key: str) -> None:
        if (self._map_q is None or self._last_fused_t is None or
                self._epoch_faulted or self._fusion_faulted):
            return
        center = self._get_map_center()
        gm = GridMap()
        gm.header.frame_id = self.map_frame
        # A TF-rejected input must not refresh the timestamp of an old map.
        gm.header.stamp = self._last_fused_t
        gm.info.resolution = self._map.resolution
        actual_map_length = (self._map.cell_n - 2) * self._map.resolution
        gm.info.length_x = actual_map_length
        gm.info.length_y = actual_map_length
        # move_to snaps XY to whole cells. The continuous robot position is not
        # the grid center. Elevation already includes the absolute center Z.
        gm.info.pose.position.x = float(center[0])
        gm.info.pose.position.y = float(center[1])
        gm.info.pose.position.z = 0.0

        gm.info.pose.orientation.x = 0.0
        gm.info.pose.orientation.y = 0.0
        gm.info.pose.orientation.z = 0.0
        gm.info.pose.orientation.w = 1.0
        gm.layers = []
        gm.basic_layers = self.my_publishers[key]["basic_layers"]

        self._map.get_raw_layers_ref(self._map_data)
        for index, layer in enumerate(("elevation", "variance")):
            gm.layers.append(layer)
            # The getter already exports GridMap axes (rows=-X, columns=-Y).
            gm.data.append(self._numpy_to_multiarray(self._map_data[index], layout="gridmap_column"))

        gm.outer_start_index = 0
        gm.inner_start_index = 0
        self._publishers_dict[key].publish(gm)

    def _get_map_center(self) -> np.ndarray:
        center = np.zeros((1, 3), dtype=np.float32)
        self._map.get_center_position(center)
        return center[0]

    def _numpy_to_multiarray(self, data: np.ndarray, layout: str = "gridmap_column") -> Float32MultiArray:
        return encode_layer_to_multiarray(data, layout=layout)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeightMappingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
