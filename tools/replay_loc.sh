#!/bin/bash
# 녹화 bag을 별도 도메인(77)에서 재생하며 localizer를 그대로 실행 → GT 대비 오차 덤프
# usage: replay_loc.sh <bagdir> <out_dump.npy> [localizer args...]   (MAP/CENTER env)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh >/dev/null 2>&1
export ROS_DOMAIN_ID=77
BAG=$1; OUT=$2; shift 2
python3 tools/localizer.py --map ${MAP:-maps/iros26_map.npz} --center ${CENTER:-maps/iros26_center.csv} --gt_check 1 --dump $OUT "$@" > logs/replay_loc.log 2>&1 &
LP=$!; sleep 2
ros2 bag play $BAG > /dev/null 2>&1
sleep 1; kill -INT $LP 2>/dev/null; sleep 1; kill $LP 2>/dev/null; wait $LP 2>/dev/null
python3 - "$OUT" <<'PY'
import sys,numpy as np
g=np.load(sys.argv[1]); t,x,y,gx,gy=g[:,:5].T; gc=g[:,10]; e=np.hypot(x-gx,y-gy); m=t>t[0]+3
ci=np.where(np.diff(gc)>0)[0]+1
print('frames %d | err mean %.3f max %.2f | >0.5m %d | collisions %d'%(m.sum(),e[m].mean(),e[m].max(),(e[m]>0.5).sum(),len(ci)))
for k in ci: print('   coll t+%.1f: err before %.2f after(+0.2s) %.2f (+1s) %.2f (+3s) %.2f'%(t[k]-t[0],e[max(k-2,0)],e[min(k+4,len(e)-1)],e[min(k+18,len(e)-1)],e[min(k+54,len(e)-1)]))
PY
