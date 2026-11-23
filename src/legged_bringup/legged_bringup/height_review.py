"""RViz start/goal -> Nav2 ComputePathToPose only; never issues FollowPath."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

class HeightReview(Node):
    def __init__(self):
        super().__init__('height_review')
        self.start_pose=None; self.busy=False
        self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.path=self.create_publisher(Path,'/legged/review_path',1)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.set_start,1)
        self.create_subscription(PoseStamped,'/goal_pose',self.plan,1)
        self.get_logger().info('Review: 2D Pose Estimate selects START, then 2D Goal Pose plans a path. No controller or cmd_vel publisher.')
    def set_start(self,msg):
        if msg.header.frame_id!='map':
            self.get_logger().error('Start must be in map frame');return
        self.start_pose=PoseStamped(header=msg.header,pose=msg.pose.pose)
        self.get_logger().info(f'Start: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
    def plan(self,msg):
        if self.start_pose is None or self.busy or msg.header.frame_id!='map':
            self.get_logger().warning('Select a map-frame start first; wait for any active plan.');return
        if not self.client.server_is_ready():
            self.get_logger().warning('Planner is still starting; select goal again.');return
        self.path.publish(Path(header=msg.header))
        goal=ComputePathToPose.Goal();goal.start=self.start_pose;goal.goal=msg
        goal.planner_id='Height';goal.use_start=True
        self.busy=True
        self.client.send_goal_async(goal).add_done_callback(self.accepted)
    def accepted(self,future):
        try:
            handle=future.result()
            if not handle.accepted:
                raise RuntimeError('Planner rejected request')
            handle.get_result_async().add_done_callback(self.finished)
        except Exception as exc:
            self.busy=False;self.get_logger().error(str(exc))
    def finished(self,future):
        self.busy=False
        try:
            result=future.result()
            if result.status!=4 or not result.result.path.poses:
                raise RuntimeError(f'No path (action status {result.status}); check planner log, unknown height and clearance.')
            self.path.publish(result.result.path)
            self.get_logger().info(f'Path: {len(result.result.path.poses)} poses')
        except Exception as exc:
            self.get_logger().error(str(exc))

def main():
    rclpy.init();node=HeightReview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
