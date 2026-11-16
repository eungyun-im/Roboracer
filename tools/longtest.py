# 직선 가감속 한계 측정: 출발선 직후 직선(남쪽)에서 풀스로틀 -> 스로틀0 / 역스로틀
import time, rclpy, numpy as np, sys
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
P='/autodrive/roboracer_1/'
rclpy.init(); n=Node('lt'); st={}
n.create_subscription(Odometry,P+'odom',lambda m:st.__setitem__('o',(time.time(),m.twist.twist.linear.x,m.pose.pose.position.y)),10)
pt=n.create_publisher(Float32,P+'throttle_command',1); ps=n.create_publisher(Float32,P+'steering_command',1); pr=n.create_publisher(Bool,'/autodrive/reset_command',1)
def reset():
    for v in (True,False):
        t0=time.time()
        while time.time()-t0<0.5: pr.publish(Bool(data=v)); rclpy.spin_once(n,timeout_sec=0.02)
def run(sec,thr,log):
    t0=time.time(); last=None
    while time.time()-t0<sec:
        pt.publish(Float32(data=float(thr))); ps.publish(Float32(data=0.0)); rclpy.spin_once(n,timeout_sec=0.01)
        if 'o' in st and st['o'] is not last: last=st['o']; log.append(last)
for thr_go,thr_stop,ty in ((1.0,0.0,'coast0'),(1.0,-1.0,'reverse')):
    reset(); run(0.8,0,[]); log=[]; run(1.0,thr_go,log); n1=len(log); run(1.2,thr_stop,log)
    a=np.array(log); keep=np.r_[True,np.diff(a[:,0])>1e-4]; n1=int(keep[:n1].sum()); a=a[keep]; t=a[:,0]-a[0,0]; v=a[:,1]; dv=np.gradient(v,t)
    print(f'[{ty}] accel phase: v 0->{v[n1-1]:.2f} in 1.0s, a_max={dv[:n1].max():.2f}, mean a(1..5m/s)={np.mean(dv[:n1][(v[:n1]>1)&(v[:n1]<5)]):.2f}, mean a(5..9)={np.mean(dv[:n1][(v[:n1]>5)&(v[:n1]<9)]) if ((v[:n1]>5)&(v[:n1]<9)).any() else float("nan"):.2f}')
    vb=v[n1:]; db=dv[n1:]; i0=np.argmax(vb<vb[0]-0.05) if (vb<vb[0]-0.05).any() else 0; print("   brake delay frames",i0, "time %.3f"%(t[n1+i0]-t[n1]))
    print(f'   brake phase: v {vb[0]:.2f}->{vb[-1]:.2f}, a_min={db.min():.2f}, mean decel(v>2)={np.mean(db[vb>2]):.2f}, dist moved y {a[0,2]:.2f}->{a[-1,2]:.2f}')
