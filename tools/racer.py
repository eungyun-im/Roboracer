#!/usr/bin/env python3
"""레이싱라인 추종: 속도연동 lookahead Pure Pursuit + 지연 보상 + 속도 프로파일.
pose_source: gt(=/odom, 디버깅 전용, 대회 런타임 금지) | loc(/racer/pose, 자체 위치추정)"""
import sys, time, argparse, json, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); from racer_core import Controller, MPCController, MPC_DEFAULTS
P='/autodrive/roboracer_1/'; L=0.324; MAXSTEER=0.5236
ap=argparse.ArgumentParser()
ap.add_argument('--line',required=True); ap.add_argument('--pose',default='gt')
ap.add_argument('--vscale',type=float,default=0.6); ap.add_argument('--ld0',type=float,default=0.5)
ap.add_argument('--ldk',type=float,default=0.15); ap.add_argument('--ldmax',type=float,default=2.5)
ap.add_argument('--lat',type=float,default=0.08)   # 지연 보상 [s]
ap.add_argument('--kthr',type=float,default=1/24.9); ap.add_argument('--laps',type=int,default=3)
ap.add_argument('--timeout',type=float,default=240); ap.add_argument('--out',default='')
ap.add_argument('--reset',type=int,default=1); ap.add_argument('--log',default=''); ap.add_argument('--vlead',type=float,default=0.25); ap.add_argument('--steer_ay',type=float,default=7.5); ap.add_argument('--steer_slip',type=float,default=3.0); ap.add_argument('--gov_e0',type=float,default=0.0); ap.add_argument('--rate_table',default='')
ap.add_argument('--ctrl',default='pp'); ap.add_argument('--vsrc',default='');  # 'gt' = 진단 전용(위치추정 모드에서 속도만 GT)
ap.add_argument('--vff',type=float,default=0.0); ap.add_argument('--aff',type=float,default=0.0); ap.add_argument('--aff_neg',type=float,default=1.0); ap.add_argument('--vm_kp',type=float,default=0.0); ap.add_argument('--vm_blend',type=float,default=0.1); ap.add_argument('--steer_abs',type=float,default=0.0); ap.add_argument('--aff_lead',type=float,default=-1.0); ap.add_argument('--rec_vmax',type=float,default=0.0); ap.add_argument('--aff_kmax',type=float,default=0.0); ap.add_argument('--arm_on_reset',type=int,default=0); ap.add_argument('--cmode',type=float,default=0); ap.add_argument('--ck',type=float,default=0.3); ap.add_argument('--cdv',type=float,default=0.3); ap.add_argument('--smode',type=float,default=0); ap.add_argument('--sband',type=float,default=0.10); ap.add_argument('--stau',type=float,default=0.15); ap.add_argument('--sk',type=float,default=0.76); ap.add_argument('--sbrk',type=float,default=0.15); ap.add_argument('--sdb',type=float,default=0.3)  # 테스트 전용: 리셋 명령(True->False) 감지 후 출발
for k in ('merge_d','afcap','afmax','af_vmin','af_dmin','qy_hi','qy_v1','qy_v2','servo_asym','N','mdelay','srate','K','qy','qpsi','rd','rref','qterm','ay_cap','cmd_lead'): ap.add_argument('--'+k,type=float,default=None)
a=ap.parse_args()
D=np.loadtxt(a.line,delimiter=','); S,X,Y,V=D[:,0],D[:,1],D[:,2],D[:,5]; M=len(X); ds=S[1]-S[0]
class R(Node):
    def __init__(s):
        super().__init__('racer'); s._bin=None; s._tbl=json.load(open(a.rate_table)) if a.rate_table else None
        kw=dict(vscale=a.vscale,ld0=a.ld0,ldk=a.ldk,ldmax=a.ldmax,lat=a.lat,vlead=a.vlead,steer_ay=a.steer_ay,steer_slip=a.steer_slip,gov_e0=a.gov_e0,vff=a.vff,aff=a.aff,aff_neg=a.aff_neg,vm_kp=a.vm_kp,vm_blend=a.vm_blend,steer_abs=a.steer_abs,aff_lead=a.aff_lead,cmode=a.cmode,ck=a.ck,cdv=a.cdv,smode=a.smode,sband=a.sband,stau=a.stau,sk=a.sk,sbrk=a.sbrk,sdb=a.sdb,aff_kmax=a.aff_kmax)
        if a.ctrl=='mpc': s.C=MPCController(a.line,**kw,**{k:getattr(a,k) for k in MPC_DEFAULTS})
        else: s.C=Controller(a.line,**kw)
        s.idx=None; s.v=0.0; s.lap=None; s.lap0=None; s.coll=0; s.coll0=None; s.laps=[]; s.err=[]; s.t0=time.time()
        s.pt=s.create_publisher(Float32,P+'throttle_command',1); s.ps=s.create_publisher(Float32,P+'steering_command',1)
        s.pr=s.create_publisher(Bool,'/autodrive/reset_command',1)
        if a.pose=='gt': s.create_subscription(Odometry,P+'odom',s.cb_odom,1)
        else:
            s.create_subscription(PoseStamped,'/racer/pose',s.cb_pose,1)
            s.create_subscription(Odometry,P+'odom' if a.vsrc=='gt' else '/racer/odom',s.cb_odom,1)
        s.create_subscription(Int32,P+'lap_count',s.cb_lap,10)
        s.recovering=False
        if a.rec_vmax>0: s.create_subscription(Bool,'/racer/recovering',lambda m:setattr(s,'recovering',m.data),1)
        if a.arm_on_reset: s.create_subscription(Bool,'/autodrive/reset_command',s.cb_reset,10)
        s.create_subscription(Int32,P+'collision_count',s.cb_coll,10)
        s.create_subscription(Float32,P+'last_lap_time',lambda m:setattr(s,'llt_msg',m.data),10)
        s.create_subscription(Float32,P+'lap_time',s.cb_lt,10)   # 집계 전용(제어 미사용): 랩 증가 직전 값=그 랩 시간
    def cb_coll(s,m):
        if a.arm_on_reset and not getattr(s,'armed',False): return
        # 리셋 시 카운터가 0으로 돌아감 -> 증가분만 누적
        if s.coll0 is not None and m.data>s.coll0: s.coll+=m.data-s.coll0
        s.coll0=m.data
    def cb_lt(s,m):
        prev=getattr(s,'lt_now',0.0)
        if m.data<prev-0.5: s.lt_done=prev   # 랩 타이머 리셋 감지 -> 직전 값이 완료 랩 시간
        s.lt_now=m.data
    def cb_lap(s,m):
        if a.arm_on_reset and not getattr(s,'armed',False): return
        if a.arm_on_reset and m.data==0: s.lap0=0; s.lap=0; return   # 리셋으로 카운터 0 복귀(랩 아님)
        if s.lap0 is None: s.lap0=m.data
        if s.lap is not None and m.data!=s.lap:
            s.llt=getattr(s,'lt_done',float('nan')) if getattr(s,'lt_now',9)<0.5 else getattr(s,'lt_now',float('nan'))
            s.laps.append((m.data, getattr(s,'llt',float('nan')), s.coll))
            print(f'lap {m.data-s.lap0} time={getattr(s,"llt",0):.3f}s coll_total={s.coll} max_err={max(s.err) if s.err else 0:.3f}',flush=True); s.err=[]
        s.lap=m.data
        if s.lap0 is not None and s.lap-s.lap0>=a.laps+(0 if a.arm_on_reset else 1): s.finish()
    def finish(s):
        s.pt.publish(Float32(data=0.0)); s.ps.publish(Float32(data=0.0))
        times=[t for (_,t,_) in s.laps] if a.arm_on_reset else ([t for (_,t,_) in s.laps[1:]] if len(s.laps)>1 else [])
        res=dict(vscale=a.vscale,ld0=a.ld0,ldk=a.ldk,lat=a.lat,laps=s.laps,timed=times,total=sum(times),coll=s.coll)
        print('RESULT',json.dumps(res),flush=True)
        if a.out: open(a.out,'a').write(json.dumps(res)+'\n')
        if a.log and hasattr(s,'L'): np.save(a.log,np.array(s.L))
        raise SystemExit
    def cb_pose(s,m):
        q=m.pose.orientation; s.pose=(m.pose.position.x,m.pose.position.y,np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))); s.control()
    def cb_odom(s,m):
        s.v=m.twist.twist.linear.x; s.wz=m.twist.twist.angular.z; s.vy=m.twist.twist.linear.y
        if a.pose=='gt':
            q=m.pose.pose.orientation; s.pose=(m.pose.pose.position.x,m.pose.pose.position.y,np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))); s.control()
    def rate_adapt(s):
        now=time.time(); lt=getattr(s,'_lt',None); s._lt=now
        if lt is None or not a.rate_table: return
        dt=min(max(now-lt,0.005),0.3); s._dt=dt if not hasattr(s,'_dt') else 0.9*s._dt+0.1*dt
        hz=1.0/s._dt; tbl=s._tbl
        k=max(i for i,e in enumerate(tbl) if hz>=e['min_hz'])
        if k!=getattr(s,'_bin',None):
            s._cand=(k,now) if getattr(s,'_cand',(None,0))[0]!=k else s._cand
            if s._bin is None or now-s._cand[1]>2.0:
                e=tbl[k]; s._bin=k; s.C=Controller(e['line'],**{x:e[x] for x in e if x not in ('min_hz','line')})
                print(f'[rate] {hz:.1f} Hz -> tier {k} ({e["line"]})',flush=True)
    def cb_reset(s,m):
        if m.data: s._rs=True
        elif getattr(s,'_rs',False) and not getattr(s,'armed',False):
            s.armed=True; s.C.reset(); s.t0=time.time(); s.lap=None; s.lap0=None; s.laps=[]; s.err=[]; s.coll=0; s.coll0=None
            print('[armed] reset detected',flush=True)
    def control(s):
        if a.arm_on_reset and not getattr(s,'armed',False): return
        s.rate_adapt()
        if time.time()-s.t0>a.timeout: print('TIMEOUT'); s.finish()
        x,y,th=s.pose; v=s.v; wz=getattr(s,'wz',0.0)
        pp=getattr(s,'_pp',None); s._pp=(x,y)
        if pp is not None and np.hypot(x-pp[0],y-pp[1])>0.3+max(v,0.5)*0.12:   # 위치 순간이동(충돌 복귀) → 제어기 상태 초기화(경로 인덱스 전역 재탐색)
            s.C.reset(); print('[racer] teleport -> controller reset',flush=True)
            if hasattr(s.C,'start_merge'): s.C.start_merge()
        # 지연 보상: 현재 속도/요레이트로 lat초 앞 위치 예측
        delta,vref,e=s.C.step(x,y,th,v,wz); s.idx=s.C.idx; s.err.append(e)
        if a.rec_vmax>0 and s.recovering: vref=min(vref,a.rec_vmax)   # 충돌복귀 직후 속도 제한(연쇄충돌 방지)
        s.pt.publish(Float32(data=float(np.clip(vref*a.kthr,0,1)))); s.ps.publish(Float32(data=float(np.clip(delta/MAXSTEER,-1,1))))
        if a.log: s.__dict__.setdefault('L',[]).append((time.time()-s.t0,s.pose[0],s.pose[1],s.pose[2],v,getattr(s,'vy',0.0),wz,delta,vref,s.idx,s.err[-1],s.coll))
rclpy.init(); n=R()
def _watchdog():   # 센서/위치 데이터가 끊기면(pose 미수신) 타임아웃 처리
    if time.time()-n.t0>a.timeout: print('TIMEOUT (no data)',flush=True); n.finish()
n.create_timer(1.0,_watchdog)
if a.reset:
    for val in (True,False):
        t0=time.time()
        while time.time()-t0<0.4: n.pr.publish(Bool(data=val)); time.sleep(0.02)
try: rclpy.spin(n)
except SystemExit: pass
