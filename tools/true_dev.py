#!/usr/bin/env python3
"""locdump(GT 위치)로 라인 대비 실제 횡이탈(속도별)과 벽 최소거리."""
import sys,numpy as np
from scipy import ndimage
dump,line,mp=sys.argv[1:4]
g=np.load(dump); t,gx,gy,gv=g[:,0],g[:,3],g[:,4],g[:,7]; m=t>t[0]+12
L=np.loadtxt(line,delimiter=','); X,Y,PS=L[:,1],L[:,2],L[:,3]
z=np.load(mp); occ=z['occ']; ox,oy=z['origin']; R=float(z['res']); DF=ndimage.distance_transform_edt(~occ)*R-R/2
dev=[]
for x,y in zip(gx[m],gy[m]):
    i=np.argmin((X-x)**2+(Y-y)**2); dev.append(-(x-X[i])*np.sin(PS[i])+(y-Y[i])*np.cos(PS[i]))
dev=np.abs(np.array(dev)); w=ndimage.map_coordinates(DF,[(gy[m]-oy)/R,(gx[m]-ox)/R],order=1); v=gv[m]
out=[]
for lo,hi in ((0,4),(4,7),(7,12)):
    s=(v>=lo)&(v<hi)
    if s.sum()>10: out.append(f'v{lo}-{hi} p95 {np.percentile(dev[s],95):.3f} p99 {np.percentile(dev[s],99):.3f}')
print('  true dev: '+' | '.join(out)+f' | wall min {w.min():.3f}')
