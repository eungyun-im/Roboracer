#!/usr/bin/env python3
"""기록 npz(GT pose+LiDAR) -> 점유격자 맵 + 중심선/트랙폭 CSV.
usage: build_track.py <rec.npz> <out_prefix>   (출력: <prefix>_map.npz, <prefix>_center.csv, <prefix>_track.png)"""
import sys, numpy as np
from scipy import ndimage
from skimage.morphology import skeletonize
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import os
RES=float(os.environ.get('MAP_RES','0.05')); LIDAR_X=float(os.environ.get('LIDAR_X','0.2733')); _SC=0.05/RES   # 격자 축소 시 같은 물리 크기 유지

def finish(occ,free,ox,oy,RES,sx,sy,syaw,pre,start=None):
    H,W=occ.shape
    edt=ndimage.distance_transform_edt(free)*RES
    sk=skeletonize(free)
    # prune spurs
    def nb(m):
        k=np.ones((3,3));k[1,1]=0; return ndimage.convolve(m.astype(int),k,mode='constant')
    for _ in range(400):
        e=sk&(nb(sk)<=1)
        if not e.any(): break
        sk&=~e
    ys,xs=np.nonzero(sk); S=set(zip(ys,xs))
    # order loop: start at point nearest start pose
    start=min(S,key=lambda p:(p[0]-sy)**2+(p[1]-sx)**2); path=[start]; seen={start}; cur=start
    while True:
        cand=[(cur[0]+a,cur[1]+b) for a in (-1,0,1) for b in (-1,0,1) if (a,b)!=(0,0) and (cur[0]+a,cur[1]+b) in S and (cur[0]+a,cur[1]+b) not in seen]
        if not cand: break
        cand.sort(key=lambda p:abs(p[0]-cur[0])+abs(p[1]-cur[1])); cur=cand[0]; path.append(cur); seen.add(cur)
    path=np.array(path); print('skeleton pts',len(path),'of',len(S))
    xy=np.c_[ox+(path[:,1]+0.5)*RES, oy+(path[:,0]+0.5)*RES]
    # orient along driving direction (start yaw)
    v0=xy[min(5,len(xy)-1)]-xy[0]
    if np.dot(v0,[np.cos(syaw),np.sin(syaw)])<0: xy=xy[::-1]
    # smooth + resample 0.1 m (periodic)
    from scipy.interpolate import splprep, splev
    xy=np.vstack([xy,xy[:1]])
    tck,u=splprep([xy[:,0],xy[:,1]],s=len(xy)*0.0004,per=1)
    L=np.sum(np.linalg.norm(np.diff(np.array(splev(np.linspace(0,1,2000),tck)).T,axis=0),axis=1))
    N=int(L/0.1); c=np.array(splev(np.linspace(0,1,N,endpoint=False),tck)).T
    cix=np.clip(((c[:,0]-ox)/RES).astype(int),0,W-1); ciy=np.clip(((c[:,1]-oy)/RES).astype(int),0,H-1)
    # widths: ray-cast along normals on free mask
    t=np.gradient(c,axis=0); t/=np.linalg.norm(t,axis=1)[:,None]; nrm=np.c_[-t[:,1],t[:,0]]
    def cast(p,dv):
        for s in np.arange(0,3.0,RES/2):
            q=p+dv*s; jx=int((q[0]-ox)/RES); jy=int((q[1]-oy)/RES)
            if jx<0 or jy<0 or jx>=W or jy>=H or not free[jy,jx]: return s
        return 3.0
    wl=np.array([cast(c[i],nrm[i]) for i in range(N)]); wr=np.array([cast(c[i],-nrm[i]) for i in range(N)])
    print(f'length {L:.2f} m, N={N}, width min {np.min(wl+wr):.2f} mean {np.mean(wl+wr):.2f} max {np.max(wl+wr):.2f}')
    np.savetxt(pre+'_center.csv',np.c_[c,wr,wl],delimiter=',',header='x_m,y_m,w_tr_right_m,w_tr_left_m',fmt='%.4f')
    np.savez_compressed(pre+'_map.npz',occ=occ,free=free,origin=np.array([ox,oy]),res=RES,start=np.array(start if start is not None else [c[0,0],c[0,1],np.arctan2(c[1,1]-c[0,1],c[1,0]-c[0,0])]))
    plt.figure(figsize=(10,12)); plt.imshow(occ,origin='lower',extent=[ox,ox+W*RES,oy,oy+H*RES],cmap='gray_r')
    plt.plot(c[:,0],c[:,1],'b-'); plt.plot(*(c+nrm*wl[:,None]).T,'g-',lw=0.5); plt.plot(*(c-nrm*wr[:,None]).T,'r-',lw=0.5)
    plt.plot(c[0,0],c[0,1],'go'); plt.savefig(pre+'_track.png',dpi=60)

if __name__=='__main__':
    d=np.load(sys.argv[1]); pre=sys.argv[2]
    R=d['ranges']; n=R.shape[1]; ang=np.radians(-135)+np.arange(n)*np.radians(270)/(n-1)
    pts=[]
    cj=np.where(np.diff(d['coll'])>0)[0]; bad=np.zeros(len(R),bool)
    for j in cj: bad[max(0,j-2):j+4]=True
    print('frames used',(~bad).sum(),'/',len(R))
    for i in np.where(~bad)[0]:
        r=R[i]; ok=(r>0.06)&(r<9.5); yw=d['yaw'][i]
        lx=d['x'][i]+LIDAR_X*np.cos(yw); ly=d['y'][i]+LIDAR_X*np.sin(yw)
        pts.append(np.c_[lx+r[ok]*np.cos(ang[ok]+yw), ly+r[ok]*np.sin(ang[ok]+yw)])
    P=np.vstack(pts)
    ox,oy=P.min(0)-1.0; W,H=((P.max(0)+1.0-[ox,oy])/RES).astype(int)+1
    hits=np.zeros((H,W),np.int32)
    ix=((P[:,0]-ox)/RES).astype(int); iy=((P[:,1]-oy)/RES).astype(int)
    np.add.at(hits,(iy,ix),1)
    occ=hits>=max(2,int(round(3/_SC)))
    occ=ndimage.binary_closing(occ,iterations=int(round(2*_SC)))|occ
    lab_o,no=ndimage.label(occ); sz=ndimage.sum(occ,lab_o,range(1,no+1))
    occ=np.isin(lab_o,1+np.where(sz>=60*_SC)[0]); print('occ components kept',(sz>=60*_SC).sum(),'of',no)
    occ_d=ndimage.binary_dilation(occ,iterations=int(round(_SC)))
    lab,_=ndimage.label(~occ_d)
    sx,sy=int((d['x'][0]-ox)/RES),int((d['y'][0]-oy)/RES)
    free=lab==lab[sy,sx]
    assert not (free[0].any() or free[-1].any() or free[:,0].any() or free[:,-1].any()), 'free space leaked to border'
    free=ndimage.binary_fill_holes(free)&~occ_d if False else free
    finish(occ,free,ox,oy,RES,sx,sy,d['yaw'][0],pre,start=[d['x'][0],d['y'][0],d['yaw'][0]])
