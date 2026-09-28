"""ROS-independent feedback-control utilities."""
import math
def wrap_angle(angle): return math.atan2(math.sin(angle),math.cos(angle))
def velocity_command(x,y,yaw,gx,gy,k_linear=0.8,k_angular=1.8,max_linear=0.45,max_angular=1.5):
    dx,dy=gx-x,gy-y; distance=math.hypot(dx,dy); target_heading=math.atan2(dy,dx); heading_error=wrap_angle(target_heading-yaw)
    angular=max(-max_angular,min(max_angular,k_angular*heading_error))
    alignment=max(0.0,math.cos(heading_error)); linear=min(max_linear,k_linear*distance)*alignment
    return linear,angular,distance,heading_error
