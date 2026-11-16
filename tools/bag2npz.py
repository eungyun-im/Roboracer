#!/usr/bin/env python3
"""bag -> npz (센서/GT 정렬 분석용). usage: bag2npz.py <bagdir> <out.npz>"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}; out={}
def st(h): return h.stamp.sec+h.stamp.nanosec*1e-9
while r.has_next():
    tp,data,tr=r.read_next(); m=deserialize_message(data,get_message(types[tp])); k=tp.split('/')[-1] if 'racer/' not in tp else 'racer_'+tp.split('/')[-1]; tr*=1e-9
    if k=='imu':
        q=m.orientation; yaw=np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        row=(tr,st(m.header),yaw,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y)
    elif k in('left_encoder','right_encoder'): row=(tr,st(m.header),m.position[0],m.velocity[0] if len(m.velocity) else np.nan)
    elif k in('odom','racer_odom'): row=(tr,st(m.header),m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)
    elif k=='racer_pose': row=(tr,st(m.header),m.pose.position.x,m.pose.position.y)
    elif k=='lidar':
        if 'lidar_meta' not in out: out['lidar_meta']=[(m.angle_min,m.angle_increment,len(m.ranges))]
        out.setdefault('lidar_r',[]).append(np.asarray(m.ranges,np.float32)); row=(tr,st(m.header))
    elif k in('collision_count','lap_count','steering_command','throttle_command','steering','throttle'): row=(tr,float(m.data))
    else: continue
    out.setdefault(k,[]).append(row)
np.savez_compressed(sys.argv[2],**{k:np.array(v) for k,v in out.items()})
print({k:np.array(v).shape for k,v in out.items()})
