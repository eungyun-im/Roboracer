#!/usr/bin/env python3
"""허용 센서만 사용하는 위치추정: IMU yaw(절대) + 엔코더 속도 추측항법 + LiDAR-맵 ICP(평행이동+미세회전) 보정.
충돌 리셋(순간이동) 시 중심선 후보 전역탐색으로 재초기화. 출력: /racer/pose, /racer/odom
usage: localizer.py --map maps/porto_map.npz --center maps/porto_center.csv [--x0 --y0]"""
import argparse, time, numpy as np, rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
import sys as _sys, os as _os; _sys.path.insert(0,_os.path.dirname(_os.path.abspath(__file__))); from tire_obs import TireObs
P='/autodrive/roboracer_1/'; RW=0.059; import os; LIDAR_X=float(os.environ.get('LIDAR_X','0.2733'))   # IPS(odom) 기준이면 0.1933 (=0.2733-0.08)
ap=argparse.ArgumentParser(); ap.add_argument('--map',required=True); ap.add_argument('--center',required=True)
ap.add_argument('--enc_scale',type=float,default=0.97); ap.add_argument('--amax',type=float,default=4.5); ap.add_argument('--amin',type=float,default=8.0); ap.add_argument('--beams',type=int,default=4)
ap.add_argument('--gt_check',type=int,default=0); ap.add_argument('--dump',default=''); ap.add_argument('--method',default='csm'); ap.add_argument('--win',type=float,default=0.25); ap.add_argument('--prior',type=float,default=0.5); ap.add_argument('--start_tol',type=float,default=0.02); ap.add_argument('--reinit_r',type=float,default=3.0); ap.add_argument('--reinit_dpen',type=float,default=0.01); ap.add_argument('--yaw_jump',type=float,default=0.2); ap.add_argument('--tele_r',type=float,default=2.0); ap.add_argument('--tele_dpen',type=float,default=0.05); ap.add_argument('--wide',type=float,default=0.0); ap.add_argument('--wide_step',type=float,default=0.4); ap.add_argument('--rec_time',type=float,default=3.0); ap.add_argument('--lon_w',type=float,default=0.0); ap.add_argument('--slip_kp',type=float,default=0.0); ap.add_argument('--slip_kn',type=float,default=0.0); ap.add_argument('--ax_tau',type=float,default=0.3); ap.add_argument('--scan_lat',type=float,default=0.0); ap.add_argument('--vobs',type=int,default=0); ap.add_argument('--r_eff',type=float,default=0.0581); ap.add_argument('--k_mu',type=float,default=1.1); ap.add_argument('--lon_lat',type=float,default=0.019); ap.add_argument('--lon_vmin',type=float,default=4.0); ap.add_argument('--lon_wzmax',type=float,default=0.5); ap.add_argument('--lon_ref',type=float,default=0.0); ap.add_argument('--lon_cos',type=float,default=0.7); ap.add_argument('--lon_min',type=int,default=12); ap.add_argument('--lon_win',type=float,default=0.4)  # 정면벽 1D 종방향 정밀정합(보정 이득)  # 가감속 슬립 보정(IMU ax)  # 진행방향 정보 점 가중(퇴화 인지)   # 1이면 GT와 오차 출력(디버깅 전용, 제어엔 미사용)
ap.add_argument('--comp',type=int,default=0)   # 1=대회 모드: GT 확인(--gt_check) 금지
ap.add_argument('--init_wait',type=float,default=0.0)   # >0: /racer/init_pose(미지 트랙 매핑→주행 인계 pose)를 최대 이 시간[s] 기다려 초기화
a=ap.parse_args()
if a.comp and a.gt_check: raise SystemExit('대회 모드(--comp 1)에서는 --gt_check 사용 불가(GT 토픽 금지)')
z=np.load(a.map); occ=z['occ']; ox,oy=z['origin']; RES=float(z['res']); START=z['start'] if 'start' in z.files else None
iy,ix=np.nonzero(occ); MP=np.c_[ox+(ix+0.5)*RES, oy+(iy+0.5)*RES]; T=cKDTree(MP)
C=np.loadtxt(a.center,delimiter=',')[:,:2]
from scipy import ndimage
FR=0.025; up=int(round(RES/FR))
occ_up=np.kron(occ,np.ones((up,up),bool))
DF=ndimage.distance_transform_edt(~occ_up)*FR
SIG=0.05; FIELD=np.exp(-0.5*(np.minimum(DF,0.3)/SIG)**2).astype(np.float32)
FH,FW=FIELD.shape
GYF,GXF=np.gradient(ndimage.gaussian_filter(DF,2),FR)   # 벽 법선(거리장 기울기)
OFF=np.arange(-a.win,a.win+1e-9,FR)   # 탐색 창
DX,DY=np.meshgrid(OFF,OFF); DX=DX.ravel(); DY=DY.ravel()
def csm(x,y,yaw,pl,prior):
    c,sn=np.cos(yaw),np.sin(yaw); W=np.c_[x+c*pl[:,0]-sn*pl[:,1], y+sn*pl[:,0]+c*pl[:,1]]
    jx=np.round((W[:,0]-ox)/FR).astype(int); jy=np.round((W[:,1]-oy)/FR).astype(int)
    k=int(round(a.win/FR)); n=len(OFF)
    # 후보 오프셋별 인덱스 = 기준 인덱스 + 정수 이동 -> (n*n, npts)
    ii=jy[None,:]+np.round(DY/FR).astype(int)[:,None]; jj=jx[None,:]+np.round(DX/FR).astype(int)[:,None]
    ok=(ii>=0)&(ii<FH)&(jj>=0)&(jj<FW)
    if a.lon_w>0:   # 점별 가중: 벽 법선이 진행방향과 나란할수록(앞/뒤 벽) 큰 가중 → 종방향 관측성 강화
        jy0=np.clip(jy,0,FH-1); jx0=np.clip(jx,0,FW-1); nx,ny=GXF[jy0,jx0],GYF[jy0,jx0]; nn=np.hypot(nx,ny)+1e-6
        wpt=1.0+a.lon_w*np.abs((nx*c+ny*sn)/nn); wpt=wpt/wpt.sum()
        sc=(np.where(ok,FIELD[np.clip(ii,0,FH-1),np.clip(jj,0,FW-1)],0)*wpt[None,:]).sum(1)
    else:
        sc=np.where(ok,FIELD[np.clip(ii,0,FH-1),np.clip(jj,0,FW-1)],0).mean(1)
    sc2=sc-prior*(DX**2+DY**2)
    b=int(np.argmax(sc2)); S=sc2.reshape(n,n); r,cc=divmod(b,n)
    ddx=ddy=0.0
    if 0<cc<n-1:
        l,m_,rr=S[r,cc-1],S[r,cc],S[r,cc+1]; den=l-2*m_+rr; ddx=0.5*(l-rr)/den*FR if den<0 else 0
    if 0<r<n-1:
        l,m_,rr=S[r-1,cc],S[r,cc],S[r+1,cc]; den=l-2*m_+rr; ddy=0.5*(l-rr)/den*FR if den<0 else 0
    return x+DX[b]+ddx, y+DY[b]+ddy, float(sc[b])
def lon_refine(x,y,yaw,pl):
    """정면/후면 벽(법선이 진행방향과 나란한) 점만으로 진행방향 1D 정합. 반환 (dx,dy,사용점수)"""
    c,sn=np.cos(yaw),np.sin(yaw); W=np.c_[x+c*pl[:,0]-sn*pl[:,1], y+sn*pl[:,0]+c*pl[:,1]]
    jx=np.clip(np.round((W[:,0]-ox)/FR).astype(int),0,FW-1); jy=np.clip(np.round((W[:,1]-oy)/FR).astype(int),0,FH-1)
    nx,ny=GXF[jy,jx],GYF[jy,jx]; nn=np.hypot(nx,ny)+1e-9; cs=np.abs((nx*c+ny*sn)/nn)
    sel=(cs>a.lon_cos)&(nn>0.2)&(pl[:,0]>1.0)   # 차 앞쪽(>1m) 정면벽만
    if sel.sum()<a.lon_min: return 0.0,0.0,int(sel.sum())
    Ws=W[sel]; offs=np.arange(-a.lon_win,a.lon_win+1e-9,FR/2)
    ii=np.round((Ws[None,:,1]+offs[:,None]*sn-oy)/FR).astype(int); jj=np.round((Ws[None,:,0]+offs[:,None]*c-ox)/FR).astype(int)
    ok=(ii>=0)&(ii<FH)&(jj>=0)&(jj<FW)
    sc=np.where(ok,FIELD[np.clip(ii,0,FH-1),np.clip(jj,0,FW-1)],0).mean(1)
    b=int(np.argmax(sc)); d=offs[b]
    if 0<b<len(offs)-1:
        l,m_,r=sc[b-1],sc[b],sc[b+1]; den=l-2*m_+r
        if den<0: d+=0.5*(l-r)/den*(offs[1]-offs[0])
    if sc[b]<0.3: return 0.0,0.0,int(sel.sum())
    return d*c,d*sn,int(sel.sum())
def yawq(q): return np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class Loc(Node):
    def __init__(s):
        super().__init__('localizer'); s.x=None; s.yaw=0.0; s.wz=0.0; s.v=0.0; s.enc=None; s.tenc=None; s.stats=[]; s.reinit=0
        s.create_subscription(Imu,P+'imu',s.cb_imu,10)
        s.create_subscription(JointState,P+'left_encoder',s.cb_el,10)
        s.create_subscription(JointState,P+'right_encoder',s.cb_er,10)
        s.create_subscription(LaserScan,P+'lidar',s.cb_scan,10)
        s.pp=s.create_publisher(PoseStamped,'/racer/pose',1); s.po=s.create_publisher(Odometry,'/racer/odom',1)
        s.prec=s.create_publisher(Bool,'/racer/recovering',1); s.rec_until=0.0   # 충돌복귀 후 복구중 신호
        s.el=s.er=None; s.ang=None; s.t_start=time.time()
        if a.init_wait>0: s.create_subscription(PoseStamped,'/racer/init_pose',s.cb_init,1)
        if a.gt_check:
            s.create_subscription(Odometry,P+'odom',lambda m:setattr(s,'gt',m),1)
            s.create_subscription(Int32,P+'collision_count',lambda m:setattr(s,'gcoll',m.data),1)
    def cb_init(s,m):
        if s.x is None: s._init=(m.pose.position.x,m.pose.position.y)
    def cb_imu(s,m):
        s.yaw=yawq(m.orientation); s.wz=m.angular_velocity.z
        al=min(1.0,0.055/max(a.ax_tau,1e-3)); s.axf=getattr(s,'axf',0.0)*(1-al)+al*m.linear_acceleration.x
    def cb_el(s,m): s.el=(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.position[0])
    def cb_er(s,m): s.er=(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.position[0])
    def speed(s):
        if s.el is None or s.er is None: return s.v
        t=s.el[0]; e=0.5*(s.el[1]+s.er[1])
        if s.enc is not None and t>s.tenc:
            de=e-s.enc
            if abs(de)<20 and a.vobs:   # 타이어 관측기: 엔코더=바퀴 표면속도 u, 차속 v는 마찰곡선+공기저항으로 적분
                if not hasattr(s,'obs'): s.obs=TireObs(a.k_mu)
                s.v=float(s.obs.step(min(t-s.tenc,0.2),de/(t-s.tenc)*a.r_eff))
            elif abs(de)<20:
                ax=getattr(s,'axf',0.0); sk=1-a.slip_kp*ax if ax>0 else 1-a.slip_kn*ax   # 가속=휠스핀(작게), 제동=잠김(크게)
                vr=de/(t-s.tenc)*RW*a.enc_scale*sk; dt=min(t-s.tenc,0.2)
                s.v=float(np.clip(vr,s.v-a.amin*dt,s.v+a.amax*dt))   # 휠스핀(가속 슬립) 제거: 물리 가속 한계로 제한
            else:
                s.need_reinit=True; s.v=0.0   # 리셋 시 엔코더 0으로 점프 -> 순간이동 신호
                s.full_reset=abs(e)<5.0
                if hasattr(s,'obs'): s.obs.init=False
        s.enc,s.tenc=e,t; return s.v
    def score(s,pts):
        d,_=T.query(pts,distance_upper_bound=0.5); return np.mean(np.minimum(d,0.5))
    def icp(s,x,y,yaw,pl):
        for it in range(6):
            c,sn=np.cos(yaw),np.sin(yaw); W=np.c_[x+c*pl[:,0]-sn*pl[:,1], y+sn*pl[:,0]+c*pl[:,1]]
            d,j=T.query(W,distance_upper_bound=0.4); ok=np.isfinite(d)
            if ok.sum()<30: return x,y,yaw,1.0,ok.mean()
            Q=MP[j[ok]]; Wk=W[ok]; ctr=np.array([x,y])
            # 2D point-to-point with small rotation (Umeyama)
            mw=Wk.mean(0); mq=Q.mean(0); H=(Wk-mw).T@(Q-mq); U,S,Vt=np.linalg.svd(H); R=Vt.T@U.T
            if np.linalg.det(R)<0: Vt[1]*=-1; R=Vt.T@U.T
            dth=np.arctan2(R[1,0],R[0,0]); dth=np.clip(dth,-0.02,0.02)
            R=np.array([[np.cos(dth),-np.sin(dth)],[np.sin(dth),np.cos(dth)]])
            tvec=mq-R@mw; newc=R@ctr+tvec; x,y=newc; yaw+=dth
            if np.hypot(*(newc-ctr))<1e-3: break
        return x,y,yaw,float(np.median(d[ok])),ok.mean()
    def global_init(s,pl,yaw,near=None,radius=None,full_reset=False,dpen=None):
        best=(1e9,None); radius=a.reinit_r if radius is None else radius; dpen=a.reinit_dpen if dpen is None else dpen
        cand=C[::2] if near is None else C[np.hypot(C[:,0]-near[0],C[:,1]-near[1])<radius]
        if len(cand)==0: cand=C[::2]
        if START is not None and full_reset: cand=np.vstack([cand,np.asarray(START[:2],float)[None,:]])   # 전체 리셋일 때만 출발점 후보
        for cx,cy in cand:
            for dx in (-0.3,0,0.3):
              for dy in (-0.3,0,0.3):
                c,sn=np.cos(yaw),np.sin(yaw); W=np.c_[cx+dx+c*pl[:,0]-sn*pl[:,1], cy+dy+sn*pl[:,0]+c*pl[:,1]]
                sc=s.score(W[::3])
                if near is not None and not full_reset: sc+=dpen*np.hypot(cx+dx-near[0],cy+dy-near[1])   # 충돌 복귀=직전 위치 근처(체크포인트) 우선
                if sc<best[0]: best=(sc,(cx+dx,cy+dy))
        if START is not None and full_reset:   # 전체 리셋(엔코더 0 초기화)일 때만 출발점 우선 — 충돌 복귀(체크포인트)엔 적용 안 함: 출발점 정합이 최고점과 거의 같으면 출발점 우선(직선 종방향 모호성 방지)
            c,sn=np.cos(yaw),np.sin(yaw); sx,sy=float(START[0]),float(START[1])
            Ws=np.c_[sx+c*pl[:,0]-sn*pl[:,1], sy+sn*pl[:,0]+c*pl[:,1]]; ss=s.score(Ws[::3])
            bx,by=best[1]
            if ss<best[0]+a.start_tol and np.hypot(bx-sx,by-sy)<2.0: best=(ss,(sx,sy))   # 출발점 근처 후보일 때만(체크포인트 복귀 시 오인 방지)
        x,y=best[1]; x,y,_,r,f=s.icp(x,y,yaw,pl); s.reinit+=1
        return x,y
    def cb_scan(s,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        r=np.asarray(m.ranges)
        if s.ang is None: s.ang=m.angle_min+np.arange(len(r))*m.angle_increment
        k=np.arange(0,len(r),a.beams); rr=r[k]; ok=np.isfinite(rr)&(rr>0.1)&(rr<9.5)
        pl=np.c_[LIDAR_X+rr[ok]*np.cos(s.ang[k][ok]), rr[ok]*np.sin(s.ang[k][ok])]
        v=s.speed(); yaw=s.yaw
        if getattr(s,'_pyaw',None) is not None and s.x is not None:
            dyaw=(yaw-s._pyaw-s.wz*max(0.0,min(t-s._pt,0.2))+np.pi)%(2*np.pi)-np.pi
            if abs(dyaw)>a.yaw_jump:   # 자이로로 설명 안 되는 방향 점프 → 체크포인트 복귀(속도 0으로 재생성됨)
                s.need_reinit=True; s.teleport=True; s.v=0.0; v=0.0; s.rec_until=t+a.rec_time
                if hasattr(s,'obs'): s.obs.v=0.0; s.obs.init=True
        s._pyaw=yaw; s._pt=t
        if s.x is None and a.init_wait>0 and time.time()-s.t_start<a.init_wait and not hasattr(s,'_init'):
            return   # 인계 pose 대기 중(엔코더 상태는 위 speed()에서 갱신)
        if s.x is None and hasattr(s,'_init'):
            s.x,s.y=map(float,s._init); s.t=t; s.start_used=True; print(f'[loc] init from handover pose ({s.x:.2f},{s.y:.2f})',flush=True)
        if s.x is None:
            if START is not None and not getattr(s,'start_used',False) and np.hypot(*(np.array([0,0]))) == 0:
                s.x,s.y=float(START[0]),float(START[1]); s.start_used=True   # 알려진 출발 pose
            else: s.x,s.y=s.global_init(pl,yaw)
            s.t=t
        else:
            dt=max(0.0,min(t-s.t,0.2)); s.t=t
            s.x+=v*dt*np.cos(yaw-s.wz*dt/2); s.y+=v*dt*np.sin(yaw-s.wz*dt/2)
        if a.method=='csm':
            x,y,scr=csm(s.x,s.y,yaw,pl,a.prior); res=1-scr; frac=scr
            if a.wide>0 and max(abs(x-s.x),abs(y-s.y))>=a.win-0.03:   # 창 끝 = 위치 놓침 의심 → 진행방향으로 넓게 재탐색
                c_,s_=np.cos(yaw),np.sin(yaw); bx,by,bs=x,y,scr
                for o in np.arange(-a.wide,a.wide+1e-9,a.wide_step):
                    if abs(o)<1e-6: continue
                    xx,yy,ss=csm(s.x+o*c_,s.y+o*s_,yaw,pl,0.0)
                    if ss>bs+0.02: bx,by,bs=xx,yy,ss
                x,y,scr=bx,by,bs; res=1-scr; frac=scr
            if a.lon_ref>0 and v>a.lon_vmin and abs(s.wz)<a.lon_wzmax:   # 직선 고속(제동 구간)에서만
                ddx,ddy,_n=lon_refine(x,y,yaw,pl)
                if _n>=a.lon_min and (ddx or ddy):   # 정면벽 정합=스캔시점 위치 → 지연(lon_lat)만큼 현재로 보정
                    ddx+=v*a.lon_lat*np.cos(yaw); ddy+=v*a.lon_lat*np.sin(yaw)
                    x+=a.lon_ref*ddx; y+=a.lon_ref*ddy
        else:
            x,y,yaw2,res,frac=s.icp(s.x,s.y,yaw,pl)
        if (a.method!='csm' and (res>0.15 or frac<0.5)) or (a.method=='csm' and scr<0.25) or getattr(s,'need_reinit',False):
            s.need_reinit=False   # 순간이동(리셋) 의심 -> 전역 재초기화
            tp=getattr(s,'teleport',False)
            gx,gy=s.global_init(pl,yaw,near=(s.x,s.y),full_reset=getattr(s,'full_reset',False),radius=(a.tele_r if tp else None),dpen=(a.tele_dpen if tp else None)); s.full_reset=False; s.teleport=False; x,y,yaw2,res,frac=s.icp(gx,gy,yaw,pl)
            if a.method=='csm': x,y,scr=csm(x,y,yaw,pl,0.0); res=1-scr
        s.x,s.y=x,y
        if a.scan_lat>0: x+=v*a.scan_lat*np.cos(yaw); y+=v*a.scan_lat*np.sin(yaw)   # 스캔 지연(~19ms) 보상: 발행 pose를 현재로
        ps=PoseStamped(); ps.header=m.header; ps.header.frame_id='world'; ps.pose.position.x=float(x); ps.pose.position.y=float(y)
        ps.pose.orientation.z=float(np.sin(yaw/2)); ps.pose.orientation.w=float(np.cos(yaw/2))
        od=Odometry(); od.header=ps.header; od.twist.twist.linear.x=float(v); od.twist.twist.angular.z=float(s.wz)
        s.po.publish(od); s.pp.publish(ps)
        s.prec.publish(Bool(data=bool(t<s.rec_until)))
        if a.gt_check and hasattr(s,'gt'):
            g=s.gt.pose.pose.position; e=np.hypot(g.x-x,g.y-y); s.stats.append(e)
            if a.dump: s.__dict__.setdefault('D',[]).append((t,x,y,g.x,g.y,yaw,v,s.gt.twist.twist.linear.x,res,frac,getattr(s,"gcoll",0)))
            if a.dump and len(s.stats)%90==0: np.save(a.dump,np.array(s.D))
            if len(s.stats)%90==0: print(f'loc err mean {np.mean(s.stats[-90:]):.3f} max {np.max(s.stats[-90:]):.3f} | v_enc {v:.2f} v_gt {s.gt.twist.twist.linear.x:.2f} | res {res:.3f} reinit {s.reinit}',flush=True)
rclpy.init(); n=Loc()
try: rclpy.spin(n)
except Exception: pass
