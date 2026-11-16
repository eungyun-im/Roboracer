#!/usr/bin/env python3
"""race_launch.identify() 시험: 각 맵의 출발 pose에서 가상 LiDAR 스캔을 만들어 트랙 판별이 맞는지 확인.
porto/iros26 → 해당 트랙, icra26(설정에 없음) → None(=explore)."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'tools'))
import race_launch as RL
from scan_sim import load_df, raycast, AMIN, AINC

cfg = RL.load_cfg(); ok = True
for mapf, want in (('maps/porto_map.npz', 'porto'), ('maps/iros26_map.npz', 'iros26'), ('maps/icra26_map.npz', None)):
    z = np.load(os.path.join(WS, mapf)); sx, sy, syaw = map(float, z['start'][:3])
    DF, ox, oy, res = load_df(os.path.join(WS, mapf))
    rng = np.random.default_rng(0)
    for dx, dy in ((0, 0), (0.05, -0.08)):   # 출발 위치가 조금 어긋나도
        r = raycast(DF, ox, oy, res, sx + dx, sy + dy, syaw) + 0.01 * rng.standard_normal(1080)
        got, sc = RL.identify(r, AMIN, AINC, syaw, cfg, verbose=False)
        print(f'{mapf} offset({dx},{dy}) -> {got}  scores {({k: round(v, 3) for k, v in sc.items()})}')
        ok &= (got == want)
print('OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)
