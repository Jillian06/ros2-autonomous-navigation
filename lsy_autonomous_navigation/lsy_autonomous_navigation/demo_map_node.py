import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class DemoMap(Node):
    def __init__(self):
        super().__init__('demo_map')
        self.pub=self.create_publisher(OccupancyGrid,'/map',10)
        self.timer=self.create_timer(0.5,self.publish_map)
    def publish_map(self):
        w=h=120; res=0.10
        data=[0]*(w*h)
        def block(x0,y0,x1,y1):
            for y in range(y0,y1):
                for x in range(x0,x1): data[y*w+x]=100
        block(0,0,w,2); block(0,h-2,w,h); block(0,0,2,h); block(w-2,0,w,h)
        block(45,15,50,82); block(72,40,77,105); block(20,70,45,75)
        msg=OccupancyGrid(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='odom'
        msg.info.resolution=res; msg.info.width=w; msg.info.height=h
        msg.info.origin.position.x=-6.0; msg.info.origin.position.y=-6.0; msg.info.origin.orientation.w=1.0
        msg.data=data; self.pub.publish(msg)
def main(args=None):
    rclpy.init(args=args); n=DemoMap(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
