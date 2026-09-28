import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from .planning import astar

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.map_msg = None
        self.odom_msg = None
        self.goal_msg = None
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

    def map_cb(self, msg):
        self.map_msg = msg
        self.try_plan()

    def odom_cb(self, msg):
        self.odom_msg = msg

    def goal_cb(self, msg):
        self.goal_msg = msg
        self.try_plan()

    def world_to_grid(self, x, y):
        info = self.map_msg.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.map_msg.info
        x = info.origin.position.x + (gx + 0.5) * info.resolution
        y = info.origin.position.y + (gy + 0.5) * info.resolution
        return x, y

    def try_plan(self):
        if self.map_msg is None or self.odom_msg is None or self.goal_msg is None:
            return
        p = self.odom_msg.pose.pose.position
        g = self.goal_msg.pose.position
        start = self.world_to_grid(p.x, p.y)
        goal = self.world_to_grid(g.x, g.y)
        info = self.map_msg.info
        cells = astar(self.map_msg.data, info.width, info.height, start, goal)
        if cells is None:
            self.get_logger().warning('No A* path found.')
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_msg.header.frame_id or 'map'
        for gx, gy in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(gx, gy)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.get_logger().info(f'Published A* path with {len(cells)} cells.')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
