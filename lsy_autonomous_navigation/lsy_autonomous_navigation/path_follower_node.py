import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from .control import velocity_command

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path = []
        self.odom = None
        self.lookahead = 0.30
        self.goal_tolerance = 0.10
        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_step)

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_cb(self, msg):
        self.odom = msg

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def choose_target(self, x, y):
        if not self.path:
            return None
        for px, py in self.path:
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.path[-1]

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_step(self):
        if self.odom is None or not self.path:
            return
        pose = self.odom.pose.pose
        x, y = pose.position.x, pose.position.y
        final_x, final_y = self.path[-1]
        if math.hypot(final_x - x, final_y - y) <= self.goal_tolerance:
            self.stop()
            self.path = []
            self.get_logger().info('Goal reached.')
            return
        target = self.choose_target(x, y)
        yaw = self.yaw_from_quaternion(pose.orientation)
        linear, angular, _, _ = velocity_command(x, y, yaw, target[0], target[1])
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
