#!/usr/bin/env python3
"""동역학 자전거모델 식별: GT twist(vx,vy,r, CoM 체좌표) + 실제 조향 → 앞/뒤 슬립각·횡력 → 타이어 곡선 μ(α)."""
import sys,numpy as np
M=3.906; L=0.324; LR=0.155; LF=L-LR; G=9.81; H=0.06
def servo_rec(tc,sc,t,delay=0.06,rate=3.2):
    # 명령→실제 조향 재구성(결승빌드: 크기 감소 즉시, 증가 rate 제한)
    a=0.0; out=np.zeros(len(t)); j=0
    for i in range(len(t)):
        dt=t[i]-t[i-1] if i else 0.055
        k=np.searchsorted(tc,t[i]-delay)-1; c=sc[k]*0.5236 if k>=0 else 0.0
        a=c if abs(c)<=abs(a) else a+np.clip(c-a,-rate*dt,rate*dt); out[i]=a
    return out
def load(f):
    z=np.load(f); od,im=z['odom'],z['imu']; t=od[:,0]
    vx,vy,r=od[:,4],od[:,5],od[:,6]
    if 'steering' in z.files: d=np.interp(t,z['steering'][:,0],z['steering'][:,1]); src='fb'
    else: sc=z['steering_command']; d=servo_rec(sc[:,0],sc[:,1],t); src='rec'
    co=z['collision_count'][:,1] if 'collision_count' in z.files else np.zeros(len(t))
    return t,vx,vy,r,d,co,src
def forces(t,vx,vy,r,d,Iz):
    ts=t; vyd=np.gradient(vy,ts); rd=np.gradient(r,ts); vxd=np.gradient(vx,ts)
    A=M*(vyd+vx*r); B=Iz*rd
    Fyf=(A*LR+B)/L/np.cos(d); Fyr=(A*LF-B)/L
    Nf=M*(G*LR-vxd*H)/L; Nr=M*(G*LF+vxd*H)/L
    af=d-np.arctan2(vy+LF*r,np.maximum(vx,0.3)); ar=-np.arctan2(vy-LR*r,np.maximum(vx,0.3))
    return af,ar,Fyf/Nf,Fyr/Nr
if __name__=='__main__':
    for f in sys.argv[1:]:
        t,vx,vy,r,d,co,src=load(f)
        good=(vx>1.5)&np.r_[True,np.diff(co)==0]&np.r_[np.diff(co)==0,True]
        good&=np.r_[True,np.abs(np.diff(vx))<1.5]
        print(f'== {f} (steer {src}) n={good.sum()}')
        best=None
        for Iz in (0.02,0.03,0.04,0.05,0.06,0.08):
            af,ar,mf,mr=forces(t,vx,vy,r,d,Iz)
            # 일관성: 같은 슬립각 구간에서 μ 분산 최소
            sc=0
            for a_,m_ in ((af,mf),(ar,mr)):
                for lo in np.arange(-0.1,0.1,0.01):
                    k=good&(a_>=lo)&(a_<lo+0.01)
                    if k.sum()>8: sc+=np.var(m_[k])*k.sum()
            if best is None or sc<best[0]: best=(sc,Iz)
        Iz=best[1]; af,ar,mf,mr=forces(t,vx,vy,r,d,Iz); print(f'  best Iz {Iz}')
        for nm,a_,m_ in (('front',af,mf),('rear',ar,mr)):
            print(f'  {nm}: alpha(deg) -> mu median [n]')
            row=[]
            for lo in (-6,-4,-3,-2,-1.5,-1,-0.5,0,0.5,1,1.5,2,3,4,6):
                hi={-6:-4,-4:-3,-3:-2,-2:-1.5,-1.5:-1,-1:-0.5,-0.5:0,0:0.5,0.5:1,1:1.5,1.5:2,2:3,3:4,4:6,6:10}[lo]
                k=good&(np.degrees(a_)>=lo)&(np.degrees(a_)<hi)
                if k.sum()>5: row.append(f'{lo}..{hi}:{np.median(m_[k]):+.2f}[{k.sum()}]')
            print('   ',' '.join(row))
