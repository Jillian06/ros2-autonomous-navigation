import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from .planning import astar
class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner'); self.map_msg=None; self.odom_msg=None; self.goal_msg=None; self.needs_plan=True
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,latched); self.create_subscription(Odometry,'/odom',self.odom_cb,10); self.create_subscription(PoseStamped,'/goal_pose',self.goal_cb,latched)
        self.path_pub=self.create_publisher(Path,'/planned_path',latched)
    def map_cb(self,msg): self.map_msg=msg; self.needs_plan=True; self.try_plan()
    def odom_cb(self,msg):
        self.odom_msg=msg
        if self.needs_plan: self.try_plan()
    def goal_cb(self,msg): self.goal_msg=msg; self.needs_plan=True; self.try_plan()
    def world_to_grid(self,x,y):
        i=self.map_msg.info; return int(math.floor((x-i.origin.position.x)/i.resolution)),int(math.floor((y-i.origin.position.y)/i.resolution))
    def grid_to_world(self,gx,gy):
        i=self.map_msg.info; return i.origin.position.x+(gx+0.5)*i.resolution,i.origin.position.y+(gy+0.5)*i.resolution
    def try_plan(self):
        if not self.needs_plan or self.map_msg is None or self.odom_msg is None or self.goal_msg is None: return
        p=self.odom_msg.pose.pose.position; g=self.goal_msg.pose.position; i=self.map_msg.info
        cells=astar(self.map_msg.data,i.width,i.height,self.world_to_grid(p.x,p.y),self.world_to_grid(g.x,g.y))
        if cells is None: self.get_logger().warning('No A* path found.'); return
        path=Path(); path.header.stamp=self.get_clock().now().to_msg(); path.header.frame_id=self.map_msg.header.frame_id or 'map'
        for gx,gy in cells:
            pose=PoseStamped(); pose.header=path.header; pose.pose.position.x,pose.pose.position.y=self.grid_to_world(gx,gy); pose.pose.orientation.w=1.0; path.poses.append(pose)
        self.path_pub.publish(path); self.needs_plan=False; self.get_logger().info(f'Published A* path with {len(cells)} cells.')
def main(args=None):
    rclpy.init(args=args); node=AStarPlannerNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
