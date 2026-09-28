import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
class DemoGoal(Node):
    def __init__(self):
        super().__init__('demo_goal'); self.pub=self.create_publisher(PoseStamped,'/goal_pose',10); self.sent=False; self.timer=self.create_timer(2.0,self.go)
    def go(self):
        if self.sent: return
        m=PoseStamped(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='odom'; m.pose.position.x=4.5; m.pose.position.y=4.0; m.pose.orientation.w=1.0
        self.pub.publish(m); self.sent=True; self.get_logger().info('Published demo goal (4.5, 4.0)')
def main(args=None):
    rclpy.init(args=args); n=DemoGoal(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
