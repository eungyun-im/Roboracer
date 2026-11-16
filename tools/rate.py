import rclpy,time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
rclpy.init(); n=Node('rate'); c=[0]
n.create_subscription(LaserScan,'/autodrive/roboracer_1/lidar',lambda m:c.__setitem__(0,c[0]+1),10)
t0=time.time()
while time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.05)
print('lidar Hz %.1f'%(c[0]/4))
