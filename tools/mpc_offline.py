#!/usr/bin/env python3
"""식별 플랜트(지연0.09, 서보3rad/s, 요1차지연0.04, K0.01, ay캡9.5, 18Hz)에서 제어기 오프라인 폐루프 평가."""
import sys,json,numpy as np; sys.path.insert(0,'tools'); from racer_core import MPCController,Controller,L
def run(line,ctrl='mpc',secs=14,hz=18.0,pd=0.09,prate=3.0,ptau=0.04,pK=0.01,seed=0,**kw):
    rng=np.random.default_rng(seed)
    C=(MPCController if ctrl=='mpc' else Controller)(line,**kw)
    D=np.loadtxt(line,delimiter=','); x,y,th=D[0,1],D[0,2],D[0,3]; v=D[0,5]; t=0.0; a=0.0; w=0.0; q=[]; E=[]; dD=[]; last=0
    h=0.005; nxt=0.0
    while t<secs:
        if t>=nxt-1e-9:
            nxt+=1/hz*(1+0.15*rng.standard_normal())
            if ctrl=='mpc': d,vr,e=C.step(x+0.005*rng.standard_normal(),y+0.005*rng.standard_normal(),th,v,w,t=t)
            else: d,vr,e=C.step(x,y,th,v,w)
            q.append((t,d)); E.append(e); dD.append(abs(d-last)); last=d
        c=0.0
        for (ts,val) in reversed(q):
            if ts<=t-pd: c=val; break
        a+=np.clip(c-a,-prate*h,prate*h)
        ws=v*np.tan(a)/(L*(1+pK*v*v)); cap=9.5/max(v,.3); ws=np.clip(ws,-cap,cap)
        w+=(ws-w)*h/ptau; x+=v*h*np.cos(th); y+=v*h*np.sin(th); th+=w*h; v+=np.clip(vr-v,-6*h,4.6*h); t+=h
    n=len(E)//4; return dict(err=float(np.mean(E[n:])),p95=float(np.percentile(E[n:],95)),dd=float(np.mean(dD[n:])))
if __name__=='__main__':
    line=sys.argv[1]; base=dict(vscale=1.0,steer_ay=8.5,steer_slip=2.5,vlead=0.15)
    print('pp',run(line,'pp',ld0=0.9,ldk=0.1,lat=0.08,**base))
    for qy,rd,md in [(8,0.5,0.13),(4,2,0.13),(2,2,0.13),(2,5,0.13),(1,3,0.13),(4,5,0.13),(2,2,0.09),(2,2,0.17)]:
        print(qy,rd,md,run(line,'mpc',qy=qy,rd=rd,mdelay=md,**base))
