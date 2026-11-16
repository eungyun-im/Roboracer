#!/usr/bin/env python3
"""구간별 속도 배율 프로파일: 라인 기하(x,y,κ)는 유지, 지정 s 구간의 횡가속 한계만 f² 배.
usage: profile_region.py <line.csv> <out.csv> --regions s0:s1:f[,..] [--alat 9.48 --alat_t 5.87 --k_t 0.8 --alat_c 4.51 --k_c 0.884 --aacc 4.5 --abrk 7]"""
import argparse, numpy as np
ap=argparse.ArgumentParser(); ap.add_argument('line'); ap.add_argument('out'); ap.add_argument('--regions',default='')
for k,v in dict(lin_a0=0.0,lin_k0=0.6,lin_slope=4.0,lin_min=6.5,alat=9.48,alat_t=5.87,k_t=0.8,alat_c=4.51,k_c=0.884,aacc=4.5,abrk=7.0,vmax=15.0,drag=0.0).items(): ap.add_argument('--'+k,type=float,default=v)
a=ap.parse_args()
D=np.loadtxt(a.line,delimiter=','); S,k=D[:,0],D[:,4]; M=len(S); ds=S[1]-S[0]; ak=np.abs(k)
if a.lin_a0>0:   # 곡률 연속 그립: κ≤k0 a0, 이후 기울기로 감소(최소 lin_min) — 코너 스윕 실측 기반
    al=np.maximum(a.lin_min,a.lin_a0-a.lin_slope*np.maximum(ak-a.lin_k0,0))
else:
    al=np.full(M,a.alat); al=np.where(ak>a.k_t,np.minimum(al,a.alat_t),al); al=np.where(ak>a.k_c,np.minimum(al,a.alat_c),al)
for r in filter(None,a.regions.split(',')):
    s0,s1,f=map(float,r.split(':')); m=(S>=s0)&(S<=s1); al[m]*=f*f
v=np.minimum(np.sqrt(al/np.maximum(ak,1e-4)),a.vmax)
for _ in range(3):
    for i in range(1,2*M):
        j=i%M; jp=(i-1)%M; ay=v[jp]**2*ak[jp]; ax=max(a.aacc-a.drag*v[jp],0.3)*np.sqrt(max(0,1-(ay/al[jp])**2)); v[j]=min(v[j],np.sqrt(v[jp]**2+2*ax*ds))
    for i in range(2*M,0,-1):
        j=(i-1)%M; jn=i%M; ay=v[jn]**2*ak[jn]; ax=(a.abrk+a.drag*v[jn])*np.sqrt(max(0,1-(ay/al[jn])**2)); v[j]=min(v[j],np.sqrt(v[jn]**2+2*ax*ds))
D[:,5]=v; np.savetxt(a.out,D,delimiter=',',header='s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps',fmt='%.5f')
print(f'est lap {np.sum(ds/v):.2f}s')
