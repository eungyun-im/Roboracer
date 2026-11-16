#!/usr/bin/env python3
"""Follow-the-gap(disparity extender) 저속 주행 + 기록. 맵 생성용(오프라인, GT pose는 학습/디버깅 용도로만 사용).
usage: ftg_record.py <out.npz> <speed m/s> <laps>"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'; K_THR=1/24.9; MAXSTEER=0.5236; L=0.324
class D(Node):
    def __init__(s,out,v,laps):
        super().__init__('ftg'); s.out=out; s.v=v; s.laps=laps; s.rec=[]; s.o=None; s.imu=None; s.enc=[None,None]
        s.lap=None; s.lap0=None; s.coll=0; s.t0=time.time()
        s.create_subscription(LaserScan,P+'lidar',s.cb_scan,10)
        s.create_subscription(Odometry,P+'odom',lambda m:setattr(s,'o',m),10)
        s.create_subscription(Imu,P+'imu',lambda m:setattr(s,'imu',m),10)
        s.create_subscription(JointState,P+'left_encoder',lambda m:s.enc.__setitem__(0,m.position[0]),10)
        s.create_subscription(JointState,P+'right_encoder',lambda m:s.enc.__setitem__(1,m.position[0]),10)
        s.create_subscription(Int32,P+'lap_count',s.cb_lap,10)
        s.create_subscription(Int32,P+'collision_count',lambda m:setattr(s,'coll',m.data),10)
        s.create_subscription(Float32,P+'last_lap_time',lambda m:setattr(s,'llt',m.data),10)
        s.pt=s.create_publisher(Float32,P+'throttle_command',10); s.ps=s.create_publisher(Float32,P+'steering_command',10)
    def cb_lap(s,m):
        if s.lap0 is None: s.lap0=m.data
        if s.lap is not None and m.data!=s.lap: print(f'lap {m.data} last_lap_time={getattr(s,"llt",0):.2f} coll={s.coll}',flush=True)
        s.lap=m.data
    def cb_scan(s,m):
        r=np.nan_to_num(np.array(m.ranges),nan=0.0,posinf=10.0); n=len(r)
        ang=m.angle_min+np.arange(n)*m.angle_increment
        # disparity extender
        rr=np.clip(r,0,10.0).copy(); half=0.20
        d=np.diff(r)
        for i in np.where(np.abs(d)>0.3)[0]:
            near=min(r[i],r[i+1]); w=int(np.ceil(np.arctan2(half,max(near,0.1))/m.angle_increment))
            if r[i]<r[i+1]: rr[i+1:i+1+w]=np.minimum(rr[i+1:i+1+w],near)
            else: rr[max(0,i-w):i+1]=np.minimum(rr[max(0,i-w):i+1],near)
        fov=np.abs(ang)<np.radians(90); cand=np.where(fov)[0]
        sm=np.convolve(rr,np.ones(15)/15,'same')
        i=cand[np.argmax(sm[cand])]; tgt=ang[i]
        steer=float(np.clip(0.9*tgt,-MAXSTEER,MAXSTEER))
        front=np.min(r[np.abs(ang)<np.radians(10)])
        v=s.v*float(np.clip((front-0.5)/3.0,0.3,1.0))
        s.pt.publish(Float32(data=float(v*K_THR))); s.ps.publish(Float32(data=float(steer/MAXSTEER)))
        if s.o is not None:
            o=s.o; p=o.pose.pose; q=p.orientation; tw=o.twist.twist
            yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            im=s.imu
            s.rec.append(dict(t=time.time()-s.t0, stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9, x=p.position.x,y=p.position.y,yaw=yaw,
                vx=tw.linear.x,vy=tw.linear.y,wz=tw.angular.z, ax=im.linear_acceleration.x if im else 0, ay=im.linear_acceleration.y if im else 0,
                gz=im.angular_velocity.z if im else 0, encL=s.enc[0] or 0, encR=s.enc[1] or 0, steer_cmd=steer, v_cmd=v, ranges=r.astype(np.float32), coll=s.coll, lap=s.lap or 0))
        if s.lap0 is not None and s.lap is not None and s.lap-s.lap0>=s.laps:
            s.save(); raise SystemExit
        if time.time()-s.t0>300: print('timeout'); s.save(); raise SystemExit
    def save(s):
        s.pt.publish(Float32(data=0.0)); s.ps.publish(Float32(data=0.0))
        keys=[k for k in s.rec[0] if k!='ranges']
        np.savez_compressed(s.out, ranges=np.stack([r['ranges'] for r in s.rec]), **{k:np.array([r[k] for r in s.rec]) for k in keys})
        print('saved',s.out,len(s.rec),'frames, collisions',s.coll)
rclpy.init(); d=D(sys.argv[1],float(sys.argv[2]),int(sys.argv[3]))
try: rclpy.spin(d)
except SystemExit: pass
