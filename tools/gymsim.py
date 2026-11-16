#!/usr/bin/env python3
"""f1tenth_gym 차량 동역학(vehicle_dynamics_st + pid, RK4 dt=0.01)을 그대로 쓰는 경량 시뮬.
AutoDRIVE 조건 모사: 18Hz 제어, 명령 지연, 위치추정 노이즈, 벽 충돌 시 +1 및 체크포인트 리셋.
usage(모듈): run(map_npz, center_csv, line_csv, laps, ctrl_params, veh_params, sim_params) -> dict"""
import sys, types, importlib.util, os, numpy as np
from scipy import ndimage
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from racer_core import Controller
# numba 스텁(사용자 site의 numba가 coverage 버전 문제로 import 실패) -> 순수 파이썬으로 실행
try:
    import numba  # noqa
except Exception:
    nb=types.ModuleType('numba'); nb.njit=lambda *a,**k:(a[0] if a and callable(a[0]) else (lambda f:f)); sys.modules['numba']=nb
spec=importlib.util.spec_from_file_location('dm',os.path.join(HERE,'..','third_party','f1tenth_gym','gym','f110_gym','envs','dynamic_models.py'))
dm=importlib.util.module_from_spec(spec); spec.loader.exec_module(dm)
# AutoDRIVE RoboRacer 제원(기술가이드) 기반 기본값; mu/C_S*/I/h는 보정 대상
VEH=dict(mu=1.0489,C_Sf=4.718,C_Sr=5.4562,lf=0.324-0.15532,lr=0.15532,h=0.074,m=3.906,I=0.06,
         s_min=-0.5236,s_max=0.5236,sv_min=-3.2,sv_max=3.2,v_switch=7.319,a_max=4.6,v_min=-5.0,v_max=22.88)
SIM=dict(dt=0.01,ctrl_hz=18.0,delay=0.10,noise_along=0.07,noise_cross=0.02,v_noise=0.03,seed=0,timeout=200.0,
         body_front=0.41,body_rear=-0.09,half_w=0.135,
         kv=0.0,a_brk=4.8,ay_cap=0.0,ay_slip=3.0,ay_collapse=0.0)   # kv>0: 1차 속도루프 대체, ay_cap>0: 타이어 포화 근사
KEYS=['mu','C_Sf','C_Sr','lf','lr','h','m','I','s_min','s_max','sv_min','sv_max','v_switch','a_max','v_min','v_max']
def load_track(map_npz,center_csv):
    z=np.load(map_npz); occ=z['occ']; ox,oy=z['origin']; R=float(z['res'])
    DF=ndimage.distance_transform_edt(~occ)*R
    C=np.loadtxt(center_csv,delimiter=',')[:,:2]
    return dict(DF=DF,ox=ox,oy=oy,R=R,C=C)
def collide(T,x,y,th,S):
    fr,re,hw=S['body_front'],S['body_rear'],S['half_w']
    pts=np.array([[fr,hw],[fr,-hw],[re,hw],[re,-hw],[fr,0],[(fr+re)/2,hw],[(fr+re)/2,-hw],[re,0]])
    c,s=np.cos(th),np.sin(th); W=np.c_[x+c*pts[:,0]-s*pts[:,1], y+s*pts[:,0]+c*pts[:,1]]
    ix=((W[:,0]-T['ox'])/T['R']).astype(int); iy=((W[:,1]-T['oy'])/T['R']).astype(int)
    H,Wd=T['DF'].shape
    if (ix<0).any() or (iy<0).any() or (ix>=Wd).any() or (iy>=H).any(): return True
    return bool((T['DF'][iy,ix]<T['R']*0.75).any())
def run(map_npz,center_csv,line_csv,laps=5,ctrl=None,veh=None,sim=None,start=None,track=None,verbose=False):
    V=dict(VEH); V.update(veh or {}); S=dict(SIM); S.update(sim or {}); rng=np.random.default_rng(S['seed'])
    T=track or load_track(map_npz,center_csv); C=T['C']; NC=len(C)
    K=Controller(line_csv,**(ctrl or {}))
    P=[V[k] for k in KEYS]
    if start is None:
        t0=C[1]-C[0]; start=(C[0,0],C[0,1],np.arctan2(t0[1],t0[0]))
    st=np.array([start[0],start[1],0.0,0.0,start[2],0.0,0.0])  # x,y,steer,v,yaw,yawrate,slip
    dt=S['dt']; nctrl=int(round(1/S['ctrl_hz']/dt)); ndelay=int(round(S['delay']/dt))
    queue=[]; cmd=(0.0,0.0); t=0.0; prog_idx=0; lapcount=-1; lap_t0=None; laps_t=[]; coll=0; step=0; maxerr=[]
    def cidx(x,y,prev):
        w=np.arange(prev-10,prev+30)%NC; return int(w[np.argmin((C[w,0]-x)**2+(C[w,1]-y)**2)])
    prog_idx=int(np.argmin((C[:,0]-st[0])**2+(C[:,1]-st[1])**2)); line_idx=(prog_idx+3)%NC; last_safe=prog_idx
    while t<S['timeout']:
        if step%nctrl==0:
            x,y,th,v,wz,beta=st[0],st[1],st[4],st[3],st[5],st[6]
            vx=v*np.cos(beta)
            ea=rng.normal(0,S['noise_along']); ec=rng.normal(0,S['noise_cross'])
            xo=x+ea*np.cos(th)-ec*np.sin(th); yo=y+ea*np.sin(th)+ec*np.cos(th)
            delta,vref,e=K.step(xo,yo,th,vx*(1+rng.normal(0,S['v_noise'])),wz); maxerr.append(e)
            queue.append((t+S['delay'],delta,vref))
        while queue and queue[0][0]<=t+1e-9: _,d_,v_=queue.pop(0); cmd=(d_,v_)
        dcmd=cmd[0]
        if S['ay_cap']>0:   # 그립 피크 조향각 초과분은 효과 없음(+붕괴): 유효 조향각으로 근사
            dpk=np.arctan(0.324*S['ay_cap']/max(st[3],0.5)**2)+np.radians(S['ay_slip'])
            if abs(dcmd)>dpk: dcmd=np.sign(dcmd)*max(dpk-(abs(dcmd)-dpk)*S['ay_collapse'],0.0)
        accl,sv=dm.pid(cmd[1],dcmd,st[3],st[2],V['sv_max'],V['a_max'],V['v_max'],V['v_min'])
        if S['kv']>0: accl=float(np.clip(S['kv']*(cmd[1]-st[3]),-S['a_brk'],V['a_max']))
        f=lambda s_:dm.vehicle_dynamics_st(s_,np.array([sv,accl]),*P)
        k1=f(st); k2=f(st+dt*k1/2); k3=f(st+dt*k2/2); k4=f(st+dt*k3); st=st+dt/6*(k1+2*k2+2*k3+k4)
        st[2]=np.clip(st[2],V['s_min'],V['s_max']); t+=dt; step+=1
        ni=cidx(st[0],st[1],prog_idx)
        # 시작선 통과(진행 방향) 검사
        if (prog_idx-line_idx)%NC>NC//2 and (ni-line_idx)%NC<NC//2 and ((ni-prog_idx)%NC)<NC//2:
            lapcount+=1
            if lap_t0 is not None: laps_t.append(t-lap_t0)
            lap_t0=t
            if verbose: print(f'lap {lapcount} t={t:.2f} coll={coll}')
            if len(laps_t)>=laps: break
        prog_idx=ni
        if step%5==0:
            if collide(T,st[0],st[1],st[4],S):
                coll+=1; j=(prog_idx-10)%NC; tg=C[(j+1)%NC]-C[j]
                st=np.array([C[j,0],C[j,1],0.0,0.0,np.arctan2(tg[1],tg[0]),0.0,0.0]); prog_idx=j; K.reset(); queue=[]; cmd=(0.0,0.0)
                if coll>30: break
    return dict(laps=laps_t,coll=coll,total=float(sum(laps_t)),done=len(laps_t)>=laps,maxerr=float(np.max(maxerr)) if maxerr else 0)
if __name__=='__main__':
    import argparse, json
    ap=argparse.ArgumentParser(); ap.add_argument('--map',default='maps/porto_map.npz'); ap.add_argument('--center',default='maps/porto_center.csv')
    ap.add_argument('--line',required=True); ap.add_argument('--laps',type=int,default=5); ap.add_argument('--vscale',type=float,default=1.0)
    ap.add_argument('--veh',default='{}'); ap.add_argument('--sim',default='{}')
    a=ap.parse_args()
    r=run(a.map,a.center,a.line,a.laps,ctrl=dict(vscale=a.vscale),veh=json.loads(a.veh),sim=json.loads(a.sim),verbose=True)
    print(json.dumps(r))
