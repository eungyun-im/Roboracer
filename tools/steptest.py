import time, rclpy, sys
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'
rclpy.init(); n=Node('t'); last={}
n.create_subscription(Odometry,P+'odom',lambda m:last.__setitem__('o',m),10)
n.create_subscription(Float32,P+'steering',lambda m:last.__setitem__('s',m.data),10)
pt=n.create_publisher(Float32,P+'throttle_command',10); ps=n.create_publisher(Float32,P+'steering_command',10)
pr=n.create_publisher(Bool,'/autodrive/reset_command',10)
def run(sec,thr,st):
    t0=time.time()
    while time.time()-t0<sec:
        pt.publish(Float32(data=float(thr))); ps.publish(Float32(data=float(st))); rclpy.spin_once(n,timeout_sec=0.02)
def reset():
    for v in (True,False):
        t0=time.time()
        while time.time()-t0<0.5: pr.publish(Bool(data=v)); rclpy.spin_once(n,timeout_sec=0.02)
reset(); run(1,0,0)
run(1.5,0.03,0.5); o=last['o']; print('steer+0.5 -> wz=%.3f steer_fb=%.3f v=%.3f'%(o.twist.twist.angular.z,last['s'],o.twist.twist.linear.x))
run(1,0,0); reset(); run(1,0,0)
for thr in (0.02,0.05,0.2):
    run(2.0,thr,0.0); print('thr %.2f -> v=%.3f'%(thr,last['o'].twist.twist.linear.x)); run(1.5,0,0); reset(); run(0.8,0,0)
