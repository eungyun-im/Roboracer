#!/usr/bin/env python3
"""종방향 회귀 시험(ROS/시뮬 없이): tools/long_sim.py(실측 랩 ±0.1s 재현하도록 보정된 1D 종방향 시뮬)에서
① 현재 대회 설정(smode 2)이 실측 근처(7.7~7.95s)인지 ② 트랙션 제어(smode 3)가 더 빠르면서 코너 그립 한계 시간을 늘리지 않는지 확인."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE); sys.path.insert(0, os.path.join(WS, 'tools')); os.chdir(WS)
import long_sim as LS

def ev(c):
    fl, ov = [], []
    for sd in (0, 1):
        r = LS.run('maps/iros26_combo.csv', c, 3, cdelay=0.165, seed=sd); fl += r['laps'][1:]; ov.append(r['over_time'])
    return float(np.mean(fl)), float(np.mean(ov))

b = ev(dict(LS.CTRL)); t = ev(dict(LS.CTRL, smode=3, sacc=0.15, apred=6.0, scap=0.5))
print(f'smode2 {b[0]:.3f}s grip-limit {100 * b[1]:.2f}% | smode3 {t[0]:.3f}s grip-limit {100 * t[1]:.2f}%')
ok = 7.7 <= b[0] <= 7.95 and t[0] < b[0] - 0.03 and t[1] <= b[1] * 1.05
print('OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)
