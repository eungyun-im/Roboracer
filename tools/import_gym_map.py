#!/usr/bin/env python3
"""f1tenth_gym 맵(yaml+png) -> 우리 형식(<prefix>_map.npz, _center.csv). 링형 자유공간을 트랙으로 선택.
usage: import_gym_map.py <map.yaml> <out_prefix>"""
import sys, os, yaml, numpy as np
from PIL import Image
from scipy import ndimage
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__))); from build_track import finish
y=yaml.safe_load(open(sys.argv[1])); pre=sys.argv[2]
im=np.array(Image.open(os.path.join(os.path.dirname(sys.argv[1]),y['image'])).convert('L')).astype(float)
im=im/im.max(); im=im[::-1]       # 원점 lower
RES=float(y['resolution']); ox,oy=y['origin'][:2]
occ=im<0.5; occ[:2,:]=True; occ[-2:,:]=True; occ[:,:2]=True; occ[:,-2:]=True
occ=ndimage.binary_dilation(occ,iterations=1)
lab,n=ndimage.label(~occ); best=None
for k in range(1,n+1):
    m=lab==k; area=m.sum()
    if area<500: continue
    if m[2].any() and m[-3].any() and m[:,2].any() and m[:,-3].any(): continue
    holes=ndimage.binary_fill_holes(m).sum()-area
    if holes>area*0.2 and (best is None or area>best[1]): best=(k,area)
assert best, 'no ring-shaped free component'
free=lab==best[0]
ys,xs=np.nonzero(free); edt=ndimage.distance_transform_edt(free)
i=np.argmax(edt[ys,xs]); sy,sx=ys[i],xs[i]
finish(occ,free,ox,oy,RES,sx,sy,0.0,pre)
