#!/usr/bin/env python3
"""토픽 주기 측정 + 짧은 구동 테스트 (throttle/steer 고정 명령)."""
import sys, time, math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'
class Probe(Node):
    def __init__(s, thr, steer, dur):
        super().__init__('probe'); s.cnt={}; s.thr=thr; s.steer=steer; s.dur=dur
        for name,typ in [('lidar',LaserScan),('imu',Imu),('odom',Odometry),('left_encoder',JointState)]:
            s.create_subscription(typ,P+name,lambda m,n=name:s.cb(n,m),10)
        s.pt=s.create_publisher(Float32,P+'throttle_command',10); s.ps=s.create_publisher(Float32,P+'steering_command',10)
        s.t0=time.time(); s.last=None; s.create_timer(0.02,s.tick)
    def cb(s,n,m):
        s.cnt[n]=s.cnt.get(n,0)+1
        if n=='odom': s.last=m
    def tick(s):
        t=time.time()-s.t0
        on = 1.0<t<1.0+s.dur
        s.pt.publish(Float32(data=s.thr if on else 0.0)); s.ps.publish(Float32(data=s.steer if on else 0.0))
        if s.last and int(t*5)!=int((t-0.02)*5):
            o=s.last; v=o.twist.twist.linear; p=o.pose.pose.position
            print(f"t={t:5.2f} x={p.x:7.3f} y={p.y:7.3f} vx={v.x:6.3f} vy={v.y:6.3f} wz={o.twist.twist.angular.z:6.3f}",flush=True)
        if t>s.dur+3:
            el=t; print({k:round(c/el,1) for k,c in s.cnt.items()},'Hz'); raise SystemExit
rclpy.init(); n=Probe(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
try: rclpy.spin(n)
except SystemExit: pass
