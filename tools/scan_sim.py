#!/usr/bin/env python3
"""오프라인 검증용 가상 센서: 점유맵(npz)에서 LiDAR(1080빔 270°) 레이캐스팅 + 엔코더/IMU 합성.
시뮬 없이 SLAM·트랙 생성 파이프라인을 시험할 때 사용(실주행 데이터 아님).
usage: scan_sim.py <map.npz> <path.csv> <out.npz> [--v 2.0 --hz 18 --laps 1.3 --enc_true 0.941 --rng_noise 0.01]
  path.csv: 중심선(x,y,...) 또는 라인(s,x,y,...). 출력 npz: t,yaw,wz,encL,encR,ranges + 참값 gx,gy,gyaw"""
import argparse, numpy as np
from scipy import ndimage
from scipy.interpolate import splprep, splev
LIDAR_X = 0.2733; RW = 0.059; NB = 1080; AMIN = np.radians(-135); AINC = np.radians(270) / (NB - 1)


def load_df(map_npz):
    z = np.load(map_npz); occ = z['occ']; ox, oy = z['origin']; res = float(z['res'])
    DF = ndimage.distance_transform_edt(~occ) * res - res / 2
    return DF, float(ox), float(oy), res


def raycast(DF, ox, oy, res, x, y, yaw, rmax=10.0):
    ang = yaw + AMIN + np.arange(NB) * AINC
    px, py = x + LIDAR_X * np.cos(yaw), y + LIDAR_X * np.sin(yaw)
    t = np.zeros(NB); done = np.zeros(NB, bool)
    for _ in range(120):
        qx = px + t * np.cos(ang); qy = py + t * np.sin(ang)
        d = ndimage.map_coordinates(DF, [(qy - oy) / res, (qx - ox) / res], order=1, mode='constant', cval=-1.0)
        hit = d < res * 0.25
        done |= hit | (t > rmax)
        if done.all(): break
        t = np.where(done, t, t + np.maximum(d, res * 0.25))
    r = np.where(t > rmax, np.inf, t)
    return r


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('map'); ap.add_argument('path'); ap.add_argument('out')
    ap.add_argument('--v', type=float, default=2.0); ap.add_argument('--hz', type=float, default=18.0)
    ap.add_argument('--laps', type=float, default=1.3); ap.add_argument('--enc_true', type=float, default=0.941)
    ap.add_argument('--enc_noise', type=float, default=0.03); ap.add_argument('--rng_noise', type=float, default=0.01)
    ap.add_argument('--wobble', type=float, default=0.15); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--start_xy', default='')   # 'x,y': 이 위치에서 정확히 출발해 2m 동안 경로로 합류(실제 차는 맵 START에서 출발)
    a = ap.parse_args(); rng = np.random.default_rng(a.seed)
    D = np.loadtxt(a.path, delimiter=',')
    xy = D[:, 1:3] if D.shape[1] >= 6 else D[:, :2]
    n0 = 0.0
    if a.start_xy:
        sx, sy = map(float, a.start_xy.split(',')); j = int(np.argmin((xy[:, 0] - sx) ** 2 + (xy[:, 1] - sy) ** 2)); xy = np.roll(xy, -j, axis=0)
        tg = xy[1] - xy[0]; nv = np.array([-tg[1], tg[0]]) / np.hypot(*tg); n0 = float(np.dot([sx - xy[0, 0], sy - xy[0, 1]], nv))   # START의 경로 법선 방향 오프셋
    q = np.vstack([xy, xy[:1]]); tk, _ = splprep([q[:, 0], q[:, 1]], s=0, per=1)
    uu = np.linspace(0, 1, 5000, endpoint=False); PP = np.array(splev(uu, tk)).T
    L = np.sum(np.linalg.norm(np.diff(np.vstack([PP, PP[:1]]), axis=0), axis=1))
    DF, ox, oy, res = load_df(a.map)
    dt = 1 / a.hz; n = int(a.laps * L / a.v / dt) + 20
    rows = {k: [] for k in ('t', 'yaw', 'wz', 'encL', 'encR', 'gx', 'gy', 'gyaw', 'v')}; R = []
    enc = 0.0; prev = None; dist = 0.0; vcur = 0.0
    for i in range(n):
        vcur = min(a.v, vcur + 3.0 * dt) if i else 0.0   # 출발 가속(3 m/s²)
        dist += vcur * dt; s = dist / L % 1.0
        x, y = splev(s, tk); dx, dy = splev(s, tk, der=1)
        nx, ny = -dy, dx; nn = np.hypot(nx, ny)
        off = a.wobble * np.sin(2 * np.pi * 3 * s) + n0 * max(0.0, 1.0 - dist / 2.0) ** 2   # FTG처럼 흔들림 + 출발 오프셋 합류
        x, y = float(x + off * nx / nn), float(y + off * ny / nn)
        yaw = float(np.arctan2(dy, dx))
        if prev is not None:
            yaw = float(np.arctan2(y - prev[1], x - prev[0]))
        wz = 0.0 if prev is None else ((yaw - prev[2] + np.pi) % (2 * np.pi) - np.pi) / dt
        v = vcur
        enc += v * dt / (RW * a.enc_true) * (1 + a.enc_noise * rng.standard_normal())
        r = raycast(DF, ox, oy, res, x, y, yaw)
        r = r + a.rng_noise * rng.standard_normal(NB)
        for k, val in zip(rows, (i * dt, yaw, wz, enc, enc, x, y, yaw, v)): rows[k].append(val)
        R.append(r.astype(np.float32)); prev = (x, y, yaw)
    np.savez_compressed(a.out, ranges=np.stack(R), angle_min=AMIN, angle_inc=AINC, **{k: np.array(v) for k, v in rows.items()})
    print(f'synthetic: L={L:.1f}m frames={n} -> {a.out}')


if __name__ == '__main__':
    main()
