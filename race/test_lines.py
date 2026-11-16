#!/usr/bin/env python3
"""레이싱라인 검증(ROS/시뮬 없이): 배포 라인(race/tracks.json)과 후보 라인이 주행 가능한지 검사.
검사: 폐곡선·균일 간격, κ 열 = 기하 곡률, 차 중심-벽 거리(계획 맵), 조향각(≤30°), 조향 속도(서보 3.2 rad/s),
      횡가속(≤ 실측 한계), 종가감속(≤ 가속/제동 한계), 속도 범위. 예상 랩도 출력.
usage: python3 race/test_lines.py [line.csv ...]   (인자 없으면 배포 + maps/porto_cand + maps/iros_cand)"""
import os, sys, glob, json
import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE)
L = 0.324; SMAX = 0.5236; SRATE = 3.2
CFG = os.environ.get('TRACKS_CFG', os.path.join(WS, 'race', 'tracks.json'))
PLAN_MAP = {n: t.get('plan_map', t['map']) for n, t in json.load(open(CFG))['tracks'].items()}   # 벽 여유는 계획용(정밀) 맵 기준
LIM = dict(wall=0.15, ay=11.0, ax_acc=8.0, ax_brk=8.6, drag=0.273, vmax=15.0)   # 제동 한계 = 최적슬립 8.5 + 공기저항 0.273·v


def track_of(path):
    """경로에 트랙 이름이 있으면 그 트랙, 모호하면 라인이 들어맞는(벽 최소거리가 큰) 맵 선택"""
    for n in sorted(PLAN_MAP, key=len, reverse=True):
        if n in os.path.basename(path) or ('/' + n) in path: return n
    if 'iros' in path and 'iros26' in PLAN_MAP: return 'iros26'
    D = np.loadtxt(path, delimiter=','); best = None
    for name, mf in PLAN_MAP.items():
        z = np.load(os.path.join(WS, mf)); ox, oy = z['origin']; R = float(z['res'])
        DF = ndimage.distance_transform_edt(~z['occ']) * R
        w = ndimage.map_coordinates(DF, [(D[:, 2] - oy) / R, (D[:, 1] - ox) / R], order=1, mode='constant', cval=-1).min()
        if best is None or w > best[0]: best = (w, name)
    return best[1]


def check(path, mapf):
    D = np.loadtxt(path, delimiter=','); errs = []; info = {}
    if D.ndim != 2 or D.shape[1] < 6: return [f'columns {D.shape}'], info
    s, x, y, k, v = D[:, 0], D[:, 1], D[:, 2], D[:, 4], D[:, 5]
    ds = np.diff(s); dsm = ds.mean()
    if ds.std() / dsm > 0.01: errs.append(f'non-uniform spacing (std/mean {ds.std() / dsm:.3f})')
    seg = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))
    if seg[-1] > 2.5 * dsm: errs.append(f'not closed (gap {seg[-1]:.3f} m)')
    # 기하 곡률(3점, 주기)
    P = np.c_[x, y]; a = P - np.roll(P, 1, 0); b = np.roll(P, -1, 0) - P; c = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    kg = 2 * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(c, axis=1))
    kd = np.abs(ndimage.uniform_filter1d(kg, 5, mode='wrap') - k)
    if np.median(kd) > 0.05: errs.append(f'kappa column inconsistent (median diff {np.median(kd):.3f})')
    z = np.load(os.path.join(WS, mapf)); occ = z['occ']; ox, oy = z['origin']; R = float(z['res'])
    DF = ndimage.distance_transform_edt(~occ) * R - R / 2
    wall = ndimage.map_coordinates(DF, [(y - oy) / R, (x - ox) / R], order=1, mode='nearest')
    if wall.min() < LIM['wall']: errs.append(f'center-wall {wall.min():.3f} m < {LIM["wall"]} at s={s[wall.argmin()]:.1f}')
    delta = np.arctan(k * L)
    if np.abs(delta).max() > SMAX: errs.append(f'steering {np.degrees(np.abs(delta).max()):.1f}° > 30°')
    rate = np.abs(np.gradient(k, dsm)) * v * L / (1 + (k * L) ** 2)
    if rate.max() > SRATE: errs.append(f'steering rate {rate.max():.2f} rad/s > {SRATE} at s={s[rate.argmax()]:.1f}')
    ay = v ** 2 * np.abs(k)
    if ay.max() > LIM['ay']: errs.append(f'lateral accel {ay.max():.2f} > {LIM["ay"]}')
    ax = v * np.gradient(v, dsm)
    brk = -(ax / (LIM['ax_brk'] + LIM['drag'] * v))
    if ax.max() > LIM['ax_acc'] or brk.max() > 1.0: errs.append(f'long accel {ax.min():.1f}..{ax.max():.1f} (brake use {brk.max():.2f} of 8.6+0.273v)')
    if v.min() <= 0 or v.max() > LIM['vmax']: errs.append(f'speed {v.min():.2f}..{v.max():.2f}')
    info = dict(est=float(np.sum(dsm / v)), wall=float(wall.min()), ay=float(ay.max()), rate=float(rate.max()), vmax=float(v.max()))
    return errs, info


def main():
    os.chdir(WS)
    if len(sys.argv) > 1: lines = sys.argv[1:]
    else:
        cfg = json.load(open(CFG))
        lines = [t['line'] for t in cfg['tracks'].values()] + sorted(glob.glob('maps/porto_cand/*.csv')) + sorted(glob.glob('maps/iros_cand/*.csv'))
    bad = 0
    print(f'{"line":42s} {"est[s]":>7s} {"wall[m]":>8s} {"ay max":>7s} {"δ̇ max":>7s} {"v max":>6s}  result')
    for ln in lines:
        errs, inf = check(ln, PLAN_MAP[track_of(ln)])
        r = 'OK' if not errs else 'FAIL: ' + '; '.join(errs); bad += bool(errs)
        print(f'{ln:42s} {inf.get("est", 0):7.3f} {inf.get("wall", 0):8.3f} {inf.get("ay", 0):7.2f} {inf.get("rate", 0):7.2f} {inf.get("vmax", 0):6.2f}  {r}')
    print('OK: all lines drivable' if not bad else f'FAIL: {bad} line(s)'); sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
