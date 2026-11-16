#!/usr/bin/env python3
"""최소시간 레이싱라인 (casadi/ipopt). 기준 라인(최소곡률)의 법선 오프셋 n과 속도 v를 동시 최적화.
제약: 벽 여유(맵 거리장), 마찰타원(가속 aacc/감속 abrk × 횡 alat(κ)), 곡률 상한 kmax. 목적: 랩타임 + 평활 정규화.
usage: mintime.py <ref_line.csv> <map.npz> <out.csv> [--margin 0.35 --alat 10.5 --alat_t 6.5 --k_t 0.8 --aacc 4.5 --abrk 7 --kmax 0.85 --step 0.2 --lam 1e-3]"""
import argparse, numpy as np, casadi as ca
from scipy import ndimage
from scipy.interpolate import splprep, splev
ap=argparse.ArgumentParser(); ap.add_argument('ref'); ap.add_argument('map'); ap.add_argument('out')
for k,v in dict(margin=0.35,alat=10.5,alat_t=6.5,k_t=0.8,aacc=4.5,abrk=7.0,vmax=15.0,kmax=0.85,step=0.2,lam=1e-3,nmax=1.5,wdk=0.0,fixn=0,zm=0.0,alat_c=0.0,k_c=0.884,lin_a0=0.0,lin_k0=0.6,lin_slope=4.0,lin_min=6.5,mg_v0=4.0,mg_vk=0.0,mg_max=0.3).items(): ap.add_argument('--'+k,type=float,default=v)
ap.add_argument('--zones',default=''); ap.add_argument('--vref_line',default='')   # 속도연동 여유용 기준 속도 라인   # 's0:s1,s0:s1' 기준라인 s 구간에서 여유 zm 적용
a=ap.parse_args()
D=np.loadtxt(a.ref,delimiter=','); p=D[:,1:3]
q=np.vstack([p,p[:1]]); tk,_=splprep([q[:,0],q[:,1]],s=0,per=1)
L=np.sum(np.linalg.norm(np.diff(np.array(splev(np.linspace(0,1,4000),tk)).T,axis=0),axis=1)); M=int(round(L/a.step))
u=np.linspace(0,1,M,endpoint=False); R=np.array(splev(u,tk)).T; T=np.array(splev(u,tk,der=1)).T; T/=np.linalg.norm(T,axis=1)[:,None]; Nn=np.c_[-T[:,1],T[:,0]]
z=np.load(a.map); occ=z['occ']; ox,oy=z['origin']; res=float(z['res'])
DF=ndimage.distance_transform_edt(~occ)*res-res/2
def df(P_): return ndimage.map_coordinates(DF,[(P_[:,1]-oy)/res,(P_[:,0]-ox)/res],order=1,mode='nearest')
# 법선 방향 허용 범위
VR=None
if a.vref_line:
    _d=np.loadtxt(a.vref_line,delimiter=','); VRx,VRy,VR=_d[:,1],_d[:,2],_d[:,5]
lo=np.zeros(M); hi=np.zeros(M); offs=np.arange(-a.nmax,a.nmax+1e-9,0.01); nfix=0
for i in range(M):
    mg=a.margin
    if a.mg_vk>0 and VR is not None:   # 속도 연동 여유: 고속 구간일수록 넓게
        vi=VR[np.argmin((VRx-R[i,0])**2+(VRy-R[i,1])**2)]; mg=float(np.clip(a.margin+a.mg_vk*max(vi-a.mg_v0,0),a.margin,a.mg_max))
    for zz in filter(None,a.zones.split(',')):
        z0,z1=map(float,zz.split(':')); si=u[i]*L
        if (z0<=si<=z1) if z0<=z1 else (si>=z0 or si<=z1): mg=max(mg,a.zm)
    ok=df(R[i]+offs[:,None]*Nn[i])>=mg
    if not ok.any(): lo[i]=hi[i]=0; continue
    # 0에 가장 가까운 허용 오프셋이 속한 연속 구간 (기준점이 여유 부족이면 벽에서 밀어냄)
    j=np.argmin(np.where(ok,np.abs(offs),1e9)); l=j; r=j
    while l>0 and ok[l-1]: l-=1
    while r<len(offs)-1 and ok[r+1]: r+=1
    lo[i],hi[i]=offs[l],offs[r]; nfix+=int(lo[i]>0 or hi[i]<0)
print(f'ref points pushed off wall: {nfix}')
if a.fixn: lo[:]=np.clip(0,lo,hi); hi[:]=lo
print(f'M={M} L={L:.2f}  free width mean {np.mean(hi-lo):.2f} min {np.min(hi-lo):.2f}')
v0=np.interp(u*L,D[:,0],D[:,5],period=L)
n=ca.MX.sym('n',M); v=ca.MX.sym('v',M)
Px=R[:,0]+n*Nn[:,0]; Py=R[:,1]+n*Nn[:,1]
def sh(x,k): return ca.vertcat(x[k:],x[:k]) if k>0 else ca.vertcat(x[M+k:],x[:M+k])
Pxn,Pyn=sh(Px,1),sh(Py,1); Pxp,Pyp=sh(Px,-1),sh(Py,-1)
ax_,ay_=Px-Pxp,Py-Pyp; bx_,by_=Pxn-Px,Pyn-Py; cx_,cy_=Pxn-Pxp,Pyn-Pyp
la=ca.sqrt(ax_**2+ay_**2); lb=ca.sqrt(bx_**2+by_**2); lc=ca.sqrt(cx_**2+cy_**2)
kap=2*(ax_*by_-ay_*bx_)/(la*lb*lc)
ds=lb; vn=sh(v,1)
akap=ca.sqrt(kap**2+1e-4)
alat=a.alat_t+(a.alat-a.alat_t)*0.5*(1-ca.tanh((akap-a.k_t)/0.05))
if a.alat_c>0: alat=alat-(a.alat_t-a.alat_c)*0.5*(1+ca.tanh((akap-a.k_c)/0.05))   # 붕괴 구간 3단계
if a.lin_a0>0:   # 곡률 연속 그립(코너 스윕 실측): softplus로 매끄럽게
    sp=0.02*ca.log(1+ca.exp((akap-a.lin_k0)/0.02)); alat=ca.fmax(a.lin_min,a.lin_a0-a.lin_slope*sp)
ay=v**2*kap; axl=(vn**2-v**2)/(2*ds)
axp=0.5*(axl+ca.sqrt(axl**2+1e-3)); axn=0.5*(-axl+ca.sqrt(axl**2+1e-3))
g=ca.vertcat((axp/a.aacc)**2+(ay/alat)**2,(axn/a.abrk)**2+(ay/alat)**2,kap)
lbg=np.r_[np.full(2*M,-np.inf),np.full(M,-a.kmax)]; ubg=np.r_[np.ones(2*M),np.full(M,a.kmax)]
d2n=sh(n,1)-2*n+sh(n,-1)
dk=sh(kap,1)-kap
J=ca.sum1(2*ds/(v+vn))+a.lam*ca.sum1(d2n**2)/a.step**4*1e-3+a.wdk*ca.sum1(dk**2)
S=ca.nlpsol('s','ipopt',{'x':ca.vertcat(n,v),'f':J,'g':g},{'ipopt.print_level':0,'print_time':0,'ipopt.max_iter':3000,'ipopt.tol':1e-6})
x0=np.r_[np.zeros(M),np.clip(v0*0.95,0.5,a.vmax)]
x0[:M]=np.clip(0,lo,hi)
r=S(x0=x0,lbx=np.r_[lo,np.full(M,0.5)],ubx=np.r_[hi,np.full(M,a.vmax)],lbg=lbg,ubg=ubg)
st=S.stats()['return_status']; x=np.array(r['x']).ravel(); nn,vv=x[:M],x[M:]
P=R+nn[:,None]*Nn
f=ca.Function('f',[n,v],[ca.sum1(2*ds/(v+vn)),kap])
t0,k0=f(np.zeros(M),x0[M:]); t1,k1=f(nn,vv)
print(f'status {st}: NLP lap ref(init v) {float(t0):.3f} -> opt {float(t1):.3f}s  |k|max {np.abs(np.array(k1)).max():.2f}  wall min {df(P).min():.3f}  n range {nn.min():+.2f}/{nn.max():+.2f}')
# 0.1m 간격 보간(s=0) 출력 -> raceline.py --from_line 평활이 기준 라인과 동일 조건
q=np.vstack([P,P[:1]]); tk2,_=splprep([q[:,0],q[:,1]],s=0,per=1)
L2=np.sum(np.linalg.norm(np.diff(np.array(splev(np.linspace(0,1,4000),tk2)).T,axis=0),axis=1)); M2=int(round(L2/0.1)); u2=np.linspace(0,1,M2,endpoint=False)
P2=np.array(splev(u2,tk2)).T; s=np.arange(M2)*L2/M2; v2=np.interp(s,np.r_[0,np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))],vv)
np.savetxt(a.out,np.c_[s,P2,np.zeros(M2),np.zeros(M2),v2],delimiter=',',header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps',fmt='%.5f')
