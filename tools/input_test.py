#!/usr/bin/env python3
"""입력 방식 실험(GT odom은 분석용): 브레이크 명령값별 감속, 스로틀값별 가속, 조향 서보 응답."""
import time, sys, json, rclpy, numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'
rclpy.init(); n=Node('inp'); last={}; rec=[]
n.create_subscription(Odometry,P+'odom',lambda m:last.__setitem__('o',m),10)
n.create_subscription(Float32,P+'steering',lambda m:last.__setitem__('s',m.data),10)
n.create_subscription(Float32,P+'throttle',lambda m:last.__setitem__('t',m.data),10)
pt=n.create_publisher(Float32,P+'throttle_command',10); ps=n.create_publisher(Float32,P+'steering_command',10)
pr=n.create_publisher(Bool,'/autodrive/reset_command',10)
def run(sec,thr,st,tag=''):
    t0=time.time(); seen=None
    while time.time()-t0<sec:
        pt.publish(Float32(data=float(thr))); ps.publish(Float32(data=float(st))); rclpy.spin_once(n,timeout_sec=0.01)
        o=last.get('o')
        if o is not None and o is not seen:
            seen=o; rec.append((tag,time.time(),o.twist.twist.linear.x,o.twist.twist.angular.z,last.get('s',0),last.get('t',0)))
def reset():
    for v in (True,False):
        t0=time.time()
        while time.time()-t0<0.5: pr.publish(Bool(data=v)); rclpy.spin_once(n,timeout_sec=0.02)
    run(1.0,0,0)
def series(tag): r=[x for x in rec if x[0]==tag]; return np.array([[x[1],x[2],x[3],x[4],x[5]] for x in r])
out={}
# 1) 가속: 스로틀값별 (1.0s)
for thr in (0.3,1.0):
    reset(); run(1.2,thr,0,f'acc{thr}'); s=series(f'acc{thr}')
    out[f'acc{thr}']=dict(v_end=round(s[-1,1],2),acc=round(float(np.polyfit(s[:,0]-s[0,0],s[:,1],1)[0]),2),thr_fb=round(float(s[-1,4]),3))
# 2) 감속: 가속 1.2s 후 브레이크 명령값별
for b in (0.0,-0.05,-0.2,-1.0):
    reset(); run(1.2,1.0,0,'pre'); run(0.8,b,0,f'brk{b}'); s=series(f'brk{b}')
    v=s[:,1]; tt=s[:,0]-s[0,0]; i=np.argmax(v<0.3) if (v<0.3).any() else len(v)-1
    dv=-np.diff(v)/np.maximum(np.diff(tt),1e-3)
    out[f'brk{b}']=dict(v0=round(v[0],2),t_to_0p3=round(tt[i],2),dec_mean=round((v[0]-v[i])/max(tt[i],1e-3),2),dec_peak=round(float(dv.max()),2),thr_fb=round(float(s[-1,4]),3))
    run(1.0,0,0)
# 3) 조향 서보 응답 (저속)
reset(); run(1.0,0.06,0,'pre2'); run(1.0,0.06,1.0,'st'); s=series('st'); tt=s[:,0]-s[0,0]
out['steer']=dict(fb=[(round(a,2),round(b,3)) for a,b in zip(tt[:12],s[:12,3])])
reset()
print(json.dumps(out,indent=1))
