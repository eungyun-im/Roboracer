#!/usr/bin/env python3
"""코너 속도 스윕 분석: 코너 구간에서 달성 횡가속(v*wz), 계획 대비, 경로오차, 슬립각, 충돌."""
import numpy as np, glob, os
REG={'C1':(16.5,21),'C2':(22,26.2),'C3':(40.2,44.3)}
for c,(s0,s1) in REG.items():
    print(f'== {c} s {s0}-{s1}')
    print('   f    v_apex(plan/act)  ay_max(plan/act p95/max)  err(mean/max)  beta_max  coll_in_corner')
    for f in ('1.0','1.06','1.12','1.18','1.24','1.3'):
        p=f'data/sweep/{c}_f{f}.npy'
        if not os.path.exists(p): continue
        d=np.load(p); L=np.loadtxt(f'maps/sweep/{c}_f{f}.csv',delimiter=','); S,K,V=L[:,0],L[:,4],L[:,5]
        t,v,vy,wz,idx,err,co=d[:,0],d[:,4],d[:,5],d[:,6],d[:,9].astype(int),d[:,10],d[:,11]
        m=(t>t[0]+10)&(S[idx]>=s0)&(S[idx]<=s1)
        if m.sum()<3: print('  ',f,'no data'); continue
        ay=np.abs(v*wz); beta=np.degrees(np.abs(np.arctan2(vy,np.maximum(v,0.5))))
        rm=(S>=s0)&(S<=s1); ia=np.argmax(np.abs(K)*rm)
        ci=np.where(np.diff(co)>0)[0]+1; cc=sum(1 for k in ci if s0-1<=S[idx[max(k-3,0)]]<=s1+1)
        print(f'  {f:4s}   {V[ia]:.2f}/{np.min(v[m]):.2f}         {np.max(V[rm]**2*np.abs(K[rm])):4.1f} / {np.percentile(ay[m],95):4.1f} {ay[m].max():4.1f}        {err[m].mean():.3f}/{err[m].max():.2f}     {beta[m].max():4.1f}      {cc} (tot {int(co[-1])})')
