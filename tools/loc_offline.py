#!/usr/bin/env python3
"""오프라인 위치추정 평가: bag npz(라이다/IMU/엔코더/GT)로 localizer의 CSM 추적을 재현, 속도별 종·횡 오차.
초기화·충돌리셋 재초기화는 GT로 대체(추적 품질만 평가). usage: loc_offline.py <bag.npz> <map.npz> <center.csv> [localizer 옵션...] [--vfuse k]"""
import sys, numpy as np
bag,mp,ct=sys.argv[1:4]; extra=sys.argv[4:]
# localizer 모듈 상단(함수/맵/필드)만 실행
src=open(__file__.replace('loc_offline.py','localizer.py')).read(); src=src[:src.index('class Loc')]
VF=0.0
if '--vfuse' in extra: i=extra.index('--vfuse'); VF=float(extra[i+1]); extra=extra[:i]+extra[i+2:]
sys.argv=['localizer.py','--map',mp,'--center',ct]+extra
G={'__name__':'locmod','__file__':__file__.replace('loc_offline.py','localizer.py')}; exec(compile(src,'localizer_top','exec'),G)
a=G['a']; csm=G['csm']; RW=G['RW']; LX=G['LIDAR_X']
z=np.load(bag); im,el,er,od,lr=z['imu'],z['left_encoder'],z['right_encoder'],z['odom'],z['lidar_r']; meta=z['lidar_meta'][0]
n=min(len(im),len(el),len(er),len(od),len(lr)); ang=meta[0]+np.arange(int(meta[2]))*meta[1]
co=z['collision_count'][:,1] if 'collision_count' in z.files else np.zeros(n)
x=y=None; v=0.0; encp=None; tp=None; out=[]; scale_adapt=1.0; axf=0.0
for i in range(n):
    t=od[i,1]; yaw=im[i,2]; wz=im[i,3]; e=0.5*(el[i,2]+er[i,2])
    al=min(1.0,0.055/max(a.ax_tau,1e-3)); axf=axf*(1-al)+al*im[i,4]
    gx,gy,gv=od[i,2],od[i,3],od[i,4]
    reset = x is None or (encp is not None and abs(e-encp)>20) or (i>0 and co[min(i,len(co)-1)]!=co[min(i-1,len(co)-1)])
    if not reset and encp is not None and t>tp:
        sk=1-a.slip_kp*axf if axf>0 else 1-a.slip_kn*axf
        dt=min(t-tp,0.2); vr=(e-encp)/(t-tp)*RW*a.enc_scale*scale_adapt*sk
        v=float(np.clip(vr,v-a.amin*dt,v+a.amax*dt))
    encp=e
    if reset: x,y=gx,gy; v=gv; tp=t; continue
    dt=max(0.0,min(t-tp,0.2)); tp=t
    px=x+v*dt*np.cos(yaw-wz*dt/2); py=y+v*dt*np.sin(yaw-wz*dt/2)
    r=lr[i].astype(float); k=np.arange(0,len(r),a.beams); rr=r[k]; ok=np.isfinite(rr)&(rr>0.1)&(rr<9.5)
    pl=np.c_[LX+rr[ok]*np.cos(ang[k][ok]), rr[ok]*np.sin(ang[k][ok])]
    cx,cy,scr=csm(px,py,yaw,pl,a.prior)
    if a.lon_ref>0 and v>a.lon_vmin and abs(wz)<a.lon_wzmax:
        ddx,ddy,_n=G['lon_refine'](cx,cy,yaw,pl)
        if _n>=a.lon_min and (ddx or ddy):
            ddx+=v*a.lon_lat*np.cos(yaw); ddy+=v*a.lon_lat*np.sin(yaw); cx+=a.lon_ref*ddx; cy+=a.lon_ref*ddy
    if VF>0:   # CSM 보정의 진행방향 성분으로 엔코더 스케일 온라인 적응
        dl=(cx-px)*np.cos(yaw)+(cy-py)*np.sin(yaw)
        if v>1.0: scale_adapt=float(np.clip(scale_adapt*(1+VF*dl/max(v*dt,1e-3)),0.9,1.1))
    x,y=cx,cy
    ex,ey=(x+v*a.scan_lat*np.cos(yaw),y+v*a.scan_lat*np.sin(yaw)) if a.scan_lat>0 else (x,y)
    dx,dy=ex-gx,ey-gy; out.append((t,gv,dx*np.cos(yaw)+dy*np.sin(yaw),-dx*np.sin(yaw)+dy*np.cos(yaw),v-gv,scale_adapt))
O=np.array(out); O=O[O[:,0]>O[0,0]+3]
print('args',' '.join(extra),'vfuse',VF,'| final scale_adapt %.3f'%O[-1,5])
for lo,hi in ((0,3),(3,5),(5,7),(7,12)):
    s=(O[:,1]>=lo)&(O[:,1]<hi)&(np.abs(O[:,2])<2)
    if s.sum()>10: print(f'  v {lo}-{hi}: lon {O[s,2].mean():+.3f}/{O[s,2].std():.3f}/p99 {np.percentile(np.abs(O[s,2]),99):.3f} | lat std {O[s,3].std():.3f} p99 {np.percentile(np.abs(O[s,3]),99):.3f} | v err {O[s,4].mean():+.2f}')
if '--dump_ts' in sys.argv or True:
    np.save(__file__.replace('tools/loc_offline.py','data/loc_offline_ts.npy'),np.array(out))
