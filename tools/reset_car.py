import time, rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
rclpy.init(); n=Node('reset_car'); pr=n.create_publisher(Bool,'/autodrive/reset_command',1)
pt=n.create_publisher(Float32,'/autodrive/roboracer_1/throttle_command',1); ps=n.create_publisher(Float32,'/autodrive/roboracer_1/steering_command',1)
for v in (True,False):
    t0=time.time()
    while time.time()-t0<0.5: pr.publish(Bool(data=v)); pt.publish(Float32(data=0.0)); ps.publish(Float32(data=0.0)); time.sleep(0.02)
time.sleep(0.5)
