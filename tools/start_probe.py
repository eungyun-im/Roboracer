#!/usr/bin/env python3
"""출발 타임라인 기록(분석 전용): reset/명령/움직임/랩 이벤트와 lap_time. usage: start_probe.py <sec> <out.npy>"""
import sys,time,numpy as np,rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Int32,Bool
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'; rclpy.init(); n=Node('probe_start'); st=dict(lt=np.nan,lc=-1,thr=0.0,v=0.0,reset=0); rec=[]; t0=time.time()
n.create_subscription(Float32,P+'lap_time',lambda m:st.__setitem__('lt',m.data),10)
n.create_subscription(Int32,P+'lap_count',lambda m:st.__setitem__('lc',m.data),10)
n.create_subscription(Float32,P+'throttle_command',lambda m:st.__setitem__('thr',m.data),10)
n.create_subscription(Bool,'/autodrive/reset_command',lambda m:st.__setitem__('reset',int(m.data)),10)
def od(m):
    st['v']=m.twist.twist.linear.x; st['x']=m.pose.pose.position.x; st['y']=m.pose.pose.position.y
    rec.append((time.time()-t0,st['lt'],st['lc'],st['thr'],st['v'],st['reset'],st['x'],st['y']))
n.create_subscription(Odometry,P+'odom',od,10)
while time.time()-t0<float(sys.argv[1]): rclpy.spin_once(n,timeout_sec=0.02)
np.save(sys.argv[2],np.array(rec)); print('saved',len(rec))
