#!/usr/bin/env python3
"""중심선 CSV -> 최소곡률 레이싱라인(반복 bounded least-squares) + 속도 프로파일(마찰원 전/후진 패스).
usage: raceline.py <center.csv> <out.csv> [--margin 0.30] [--alat 6] [--aacc 6] [--abrk 8] [--vmax 15]"""
import sys, argparse, numpy as np
from scipy.optimize import lsq_linear
from scipy.interpolate import splprep, splev
ap=argparse.ArgumentParser(); ap.add_argument('center'); ap.add_argument('out')
ap.add_argument('--margin',type=float,default=0.30)   # 벽까지 최소 여유(차폭 절반 0.135 포함)
ap.add_argument('--alat',type=float,default=6.0); ap.add_argument('--aacc',type=float,default=6.0)
ap.add_argument('--abrk',type=float,default=8.0); ap.add_argument('--vmax',type=float,default=15.0)
ap.add_argument('--ds',type=float,default=0.1); ap.add_argument('--alat_tight',type=float,default=0); ap.add_argument('--k_tight',type=float,default=0.8); ap.add_argument('--iters',type=int,default=4)
ap.add_argument('--mode',default='mincurv'); ap.add_argument('--kmax',type=float,default=0.0); ap.add_argument('--from_line',default=''); ap.add_argument('--band_map',default=''); ap.add_argument('--band_w',type=float,default=1.0); ap.add_argument('--smooth',type=float,default=3.0); ap.add_argument('--resmooth',type=float,default=0.0005); ap.add_argument('--alat_collapse',type=float,default=0); ap.add_argument('--k_collapse',type=float,default=0.884)  # 조향16° 초과(tanδ=Lκ) 타이어 붕괴 구간 그립  # mincurv | center
a=ap.parse_args()
C=np.loadtxt(a.center,delimiter=','); c=C[:,:2]; wr=C[:,2]; wl=C[:,3]; N=len(c)
def normals(p):
    t=np.roll(p,-1,0)-np.roll(p,1,0); t/=np.linalg.norm(t,axis=1)[:,None]; return np.c_[-t[:,1],t[:,0]]
def curvature(p):
    d1=(np.roll(p,-1,0)-np.roll(p,1,0))/2; d2=np.roll(p,-1,0)-2*p+np.roll(p,1,0)
    return (d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.maximum(np.linalg.norm(d1,axis=1)**3,1e-9)
from scipy.ndimage import minimum_filter1d, uniform_filter1d
# 중심선/폭 평활화(벽 노이즈 → 레이싱라인 꺾임 방지)
q0=np.vstack([c,c[:1]]); tck0,_=splprep([q0[:,0],q0[:,1]],s=N*0.003,per=1); c=np.array(splev(np.linspace(0,1,N,endpoint=False),tck0)).T
wr=uniform_filter1d(minimum_filter1d(wr,9,mode='wrap'),9,mode='wrap'); wl=uniform_filter1d(minimum_filter1d(wl,9,mode='wrap'),9,mode='wrap')
nrm=normals(c); alpha=np.zeros(N)
lo=-(wr-a.margin); hi=(wl-a.margin)
bad=lo>hi; lo[bad]=hi[bad]=((wl-wr)/2)[bad]
import osqp, scipy.sparse as sp
def qp_box(A,b,lo,hi,x0):
    # min ||A x - b||^2  s.t. lo<=x<=hi  (OSQP)
    Pm=sp.csc_matrix(2*A.T@A); q=-2*A.T@b
    m=osqp.OSQP(); m.setup(P=Pm,q=q,A=sp.eye(len(lo),format='csc'),l=lo,u=hi,verbose=False,eps_abs=1e-7,eps_rel=1e-7,max_iter=20000,polish=True)
    m.warm_start(x=x0); r=m.solve(); return np.clip(r.x,lo,hi)
def solve_qp(lo,hi,alpha0):
    I=np.eye(N); D2=np.roll(I,-1,1)-2*I+np.roll(I,1,1); alpha=alpha0
    for it in range(a.iters):
        # p = c + n*alpha ; minimize ||D2 p||^2 (2차차분 ~ 곡률*ds^2)
        Ax=D2*nrm[:,0][None,:]; Ay=D2*nrm[:,1][None,:]
        D3=np.roll(D2,-1,1)-D2; lam=a.smooth
        A=np.vstack([Ax,Ay,lam*D3*nrm[:,0][None,:],lam*D3*nrm[:,1][None,:]]); b=-np.r_[D2@c[:,0],D2@c[:,1],lam*D3@c[:,0],lam*D3@c[:,1]]
        alpha=qp_box(A,b,lo,hi,alpha)
    return alpha
if a.mode=='mincurv' and not a.from_line:
    alpha=solve_qp(lo,hi,alpha)
    if a.kmax>0:   # 곡률 상한: 최종 리샘플 라인 기준으로 초과 구간 안쪽 경계를 밀어내며 재풀이
        from scipy.spatial import cKDTree
        lo2,hi2=lo.copy(),hi.copy()
        for rep in range(40):
            p_=c+nrm*alpha[:,None]; q_=np.vstack([p_,p_[:1]]); tk,_=splprep([q_[:,0],q_[:,1]],s=len(q_)*0.0005,per=1)
            uu=np.linspace(0,1,1500,endpoint=False); PP=np.array(splev(uu,tk)).T; e1=np.array(splev(uu,tk,der=1)).T; e2=np.array(splev(uu,tk,der=2)).T
            kk=(e1[:,0]*e2[:,1]-e1[:,1]*e2[:,0])/np.linalg.norm(e1,axis=1)**3
            bad=np.where(np.abs(kk)>a.kmax)[0]
            if len(bad)==0: break
            _,ci=cKDTree(p_).query(PP[bad])
            for i,sg in zip(ci,np.sign(kk[bad])):
                w=np.arange(i-6,i+7)%N
                if sg>0: hi2[w]=np.maximum(hi2[w]-0.03,lo2[w]+0.02)
                else: lo2[w]=np.minimum(lo2[w]+0.03,hi2[w]-0.02)
            alpha=solve_qp(lo2,hi2,alpha)
        print(f'kmax pass: reps={rep}, remaining over-limit samples={len(bad)}')
p=c+nrm*alpha[:,None]
if a.from_line:
    p=np.loadtxt(a.from_line,delimiter=',')[:,1:3]; a.band_map=''
if a.band_map:   # 고무줄 정제: 거리장(DF)으로 벽 여유를 직접 제약하며 2D 자유 좌표에서 곡률 최소화
    from scipy import ndimage
    from scipy.optimize import minimize
    z=np.load(a.band_map); occ=z['occ']; ox,oy=z['origin']; R=float(z['res'])
    DF=ndimage.distance_transform_edt(~occ)*R - R/2   # 벽 셀 경계까지 거리
    gy_,gx_=np.gradient(DF,R)
    def samp(F,P_): return ndimage.map_coordinates(F,[(P_[:,1]-oy)/R,(P_[:,0]-ox)/R],order=1,mode='nearest')
    # 균일 간격 초기화
    q0=np.vstack([p,p[:1]]); tk0,_=splprep([q0[:,0],q0[:,1]],s=len(q0)*0.0005,per=1)
    Lb=np.sum(np.linalg.norm(np.diff(np.array(splev(np.linspace(0,1,3000),tk0)).T,axis=0),axis=1)); Nb=int(Lb/0.1)
    P0=np.array(splev(np.linspace(0,1,Nb,endpoint=False),tk0)).T
    mg=a.margin
    def E(v):
        P_=v.reshape(-1,2); d2=np.roll(P_,-1,0)-2*P_+np.roll(P_,1,0); d3=np.roll(d2,-1,0)-d2
        sg=np.roll(P_,-1,0)-P_; ln=np.linalg.norm(sg,axis=1); lm=ln.mean()
        df=samp(DF,P_); viol=np.maximum(mg-df,0)
        e=np.sum(d2**2)/lm**4*a.band_w*1e-2 + np.sum(d3**2)/lm**6*a.smooth*1e-4 + 50*np.sum((ln-lm)**2)/lm**2 + 2e4*np.sum(viol**2)
        g=np.zeros_like(P_)
        # grad of curvature term: D2^T D2 P
        g+= (np.roll(d2,-1,0)-2*d2+np.roll(d2,1,0))*2/lm**4*a.band_w*1e-2
        g+= -(d3-np.roll(d3,1,0)) if False else 0
        dd3=np.roll(d3,1,0)-d3   # D3^T d3 (approx)
        g+= (np.roll(dd3,-1,0)-2*dd3+np.roll(dd3,1,0))*2/lm**6*a.smooth*1e-4
        r_=(ln-lm)/np.maximum(ln,1e-6); gs=sg*r_[:,None]*2*50/lm**2; g+= -gs+np.roll(gs,1,0)
        gd=np.c_[samp(gx_,P_),samp(gy_,P_)]; g+= -2e4*2*viol[:,None]*gd
        return e,g.ravel()
    r=minimize(E,P0.ravel(),jac=True,method='L-BFGS-B',options=dict(maxiter=3000))
    p=r.x.reshape(-1,2); print(f'band: {r.message}, min wall clearance {samp(DF,p).min():.3f}m (margin {mg})')
# resample uniform ds
q=np.vstack([p,p[:1]]); tck,_=splprep([q[:,0],q[:,1]],s=len(q)*a.resmooth,per=1)
L=np.sum(np.linalg.norm(np.diff(np.array(splev(np.linspace(0,1,4000),tck)).T,axis=0),axis=1))
M=int(round(L/a.ds)); u=np.linspace(0,1,M,endpoint=False); P=np.array(splev(u,tck)).T
d1=np.array(splev(u,tck,der=1)).T; d2=np.array(splev(u,tck,der=2)).T
k=(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.linalg.norm(d1,axis=1)**3
ds=L/M
# velocity profile
al=np.full(M,a.alat)
if a.alat_tight>0: al=np.where(np.abs(k)>a.k_tight,np.minimum(al,a.alat_tight),al)   # 큰 조향각 구간 그립 저하 반영
if a.alat_collapse>0: al=np.where(np.abs(k)>a.k_collapse,np.minimum(al,a.alat_collapse),al)
v=np.minimum(np.sqrt(al/np.maximum(np.abs(k),1e-4)),a.vmax)
for _ in range(3):
    for i in range(1,2*M):   # forward (accel, friction circle)
        j=i%M; jp=(i-1)%M; alat=v[jp]**2*abs(k[jp]); ax=a.aacc*np.sqrt(max(0,1-(alat/a.alat)**2))
        v[j]=min(v[j],np.sqrt(v[jp]**2+2*ax*ds))
    for i in range(2*M,0,-1):  # backward (brake)
        j=(i-1)%M; jn=i%M; alat=v[jn]**2*abs(k[jn]); ax=a.abrk*np.sqrt(max(0,1-(alat/a.alat)**2))
        v[j]=min(v[j],np.sqrt(v[jn]**2+2*ax*ds))
s=np.arange(M)*ds; lap=np.sum(ds/v)
yaw=np.arctan2(d1[:,1],d1[:,0])
np.savetxt(a.out,np.c_[s,P,yaw,k,v],delimiter=',',header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps',fmt='%.5f')
print(f'[{a.mode}] L={L:.2f}m  |k|max={np.abs(k).max():.2f}  v min/mean/max={v.min():.2f}/{v.mean():.2f}/{v.max():.2f}  est lap={lap:.2f}s')
