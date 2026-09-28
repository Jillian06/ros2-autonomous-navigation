import math
from autonomous_navigation.control import wrap_angle, velocity_command
def test_wrap_angle(): assert math.isclose(wrap_angle(3*math.pi),math.pi,abs_tol=1e-9)
def test_forward_command():
    linear,angular,distance,_=velocity_command(0,0,0,1,0); assert linear>0; assert abs(angular)<1e-9; assert math.isclose(distance,1.0)
def test_turn_before_driving_backward():
    linear,angular,_,_=velocity_command(0,0,0,-1,0); assert linear==0.0; assert abs(angular)>0
