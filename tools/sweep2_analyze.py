#!/usr/bin/env python3
"""sweep2 분석: 코너 구간별·곡률대별 계획/달성 횡가속, 최대오차, 코너내 충돌."""
import numpy as np, os, sys
CASES=[('base_asym0','base',None),('base_asym1','base',None)]+[(f'M1_f{f}_c{c}',f'M1_f{f}',(22,26)) for f in ('1.04','1.08') for c in (0,1,2)]+[(f'H_f{f}',f'H_f{f}',None) for f in ('1.1','1.2','1.3')]
REG={'H1':(16.5,21),'M1':(22,26),'M2':(35.5,39),'H2':(40.3,44.2)}
for name,prof,_ in CASES:
    p=f'data/sweep2/{name}.npy'
    if not os.path.exists(p): continue
    d=np.load(p); L=np.loadtxt(f'maps/sweep2/{prof}.csv',delimiter=','); S,K,V=L[:,0],np.abs(L[:,4]),L[:,5]
    t,v,wz,idx,err,co=d[:,0],d[:,4],d[:,6],d[:,9].astype(int),d[:,10],d[:,11]
    m0=t>t[0]+11; ci=np.where(np.diff(co)>0)[0]+1
    out=[]
    for r,(s0,s1) in REG.items():
        m=m0&(S[idx]>=s0)&(S[idx]<=s1)
        if m.sum()<3: continue
        apex=m&(K[idx]>0.8*K[(S>=s0)&(S<=s1)].max())
        cc=sum(1 for k in ci if s0-0.5<=S[idx[max(k-3,0)]]<=s1+0.5)
        out.append(f'{r}: plan {np.median(V[idx[apex]]**2*K[idx[apex]]):4.1f} act {np.median(np.abs(v[apex]*wz[apex])):4.1f} err {err[m].max():.2f} c{cc}')
    print(f'{name:14s} coll {int(co[-1])} | '+' | '.join(out))
