#!/usr/bin/env python3
"""race_launch.identify() 시험: tracks.json 의 모든 트랙 출발 pose에서 가상 LiDAR 스캔을 만들어 트랙 판별이 맞는지 확인.
각 트랙 → 자기 자신, icra26(설정에 없음) → None(=explore). 새 트랙 등록(track_add.py) 후 다시 돌려 구분되는지 확인."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(WS, 'tools'))
import race_launch as RL
from scan_sim import load_df, raycast, AMIN, AINC

cfg = RL.load_cfg(); ok = True
cases = [(t['map'], n) for n, t in cfg['tracks'].items()]
if not any('icra26' in m for m, _ in cases): cases.append(('maps/icra26_map.npz', None))   # 설정에 없는 트랙 → explore
for mapf, want in cases:
    z = np.load(os.path.join(WS, mapf)); sx, sy, syaw = map(float, z['start'][:3])
    DF, ox, oy, res = load_df(os.path.join(WS, mapf))
    rng = np.random.default_rng(0)
    for dx, dy in ((0, 0), (0.05, -0.08)):   # 출발 위치가 조금 어긋나도
        r = raycast(DF, ox, oy, res, sx + dx, sy + dy, syaw) + 0.01 * rng.standard_normal(1080)
        got, sc = RL.identify(r, AMIN, AINC, syaw, cfg, verbose=False)
        print(f'{mapf} offset({dx},{dy}) -> {got}  scores {({k: round(v, 3) for k, v in sc.items()})}')
        ok &= (got == want)
print('OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)
