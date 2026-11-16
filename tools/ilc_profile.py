#!/usr/bin/env python3
"""구간별 학습형 속도 보정(ILC): 반복 주행 로그로 구간마다 속도(횡그립) 배율을 조금씩 조정.
같은 트랙을 반복 주행하는 점을 이용 — 여유 있는 구간은 빠르게, 충돌/근접 구간은 느리게. 손으로 하던 구간 배율 튜닝 자동화.
입력 로그: racer.py --log <npy> (열: t,x,y,th,v,vy,wz,delta,vref,idx,err,coll). 연습 주행 분석용(대회 런타임 아님).
usage:
  ilc_profile.py init  <state.json> --base maps/iros26_final.csv --profile "--lin_a0 9.2 --lin_slope 2.5 --lin_min 6.5 --aacc 6.0 --abrk 8.5 --drag 0.273" [--bin 2.0]
  ilc_profile.py update <state.json> <log.npy> [<log2.npy> ...] --map maps/r025/iros26_map.npz --out <line.csv>
  ilc_profile.py show  <state.json>
규칙(구간별): 충돌 → f -= 0.03×충돌수(최대 0.06, 직전 구간도 절반) / 벽여유<c_lo → f -= 0.01 /
            모든 통과에서 벽여유>c_hi 이고 이탈 p95<e_hi → f += 0.015. f ∈ [fmin, fmax], 이웃 구간 평활.
벽여유 = 위치(로그 pose)에서 맵 벽까지 거리 - 반폭(0.135). 테스트 모드 로그의 pose는 위치추정 값(GT 아님)."""
import argparse, json, os, sys, subprocess, shlex
import numpy as np
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__))


def bins_of(S, Ltot, b):
    nb = max(1, int(round(Ltot / b))); return nb, np.minimum((S / Ltot * nb).astype(int), nb - 1)


def write_line(st, out):
    D = np.loadtxt(st['base'], delimiter=','); Ltot = D[-1, 0] + (D[1, 0] - D[0, 0]); nb = len(st['f'])
    centers = (np.arange(nb) + 0.5) * Ltot / nb
    fac = out + '.factors.csv'; np.savetxt(fac, np.c_[centers, st['f']], delimiter=',', fmt='%.4f')
    r = subprocess.run([sys.executable, os.path.join(HERE, 'profile_region.py'), st['base'], out, '--factors', fac] + shlex.split(st['profile']),
                       capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('cmd'); ap.add_argument('state'); ap.add_argument('logs', nargs='*')
    ap.add_argument('--base'); ap.add_argument('--profile', default='--lin_a0 9.2 --lin_slope 2.5 --lin_min 6.5 --aacc 6.0 --abrk 8.5 --drag 0.273')
    ap.add_argument('--bin', type=float, default=2.0); ap.add_argument('--map'); ap.add_argument('--out')
    ap.add_argument('--half_w', type=float, default=0.135); ap.add_argument('--c_lo', type=float, default=0.02); ap.add_argument('--c_hi', type=float, default=0.06)
    ap.add_argument('--e_hi', type=float, default=0.06); ap.add_argument('--up', type=float, default=0.015); ap.add_argument('--down', type=float, default=0.03)
    ap.add_argument('--fmin', type=float, default=0.85); ap.add_argument('--fmax', type=float, default=1.12); ap.add_argument('--skip_t', type=float, default=0.0)
    ap.add_argument('--lookback', type=float, default=0.3)
    a = ap.parse_args()
    if a.cmd == 'init':
        D = np.loadtxt(a.base, delimiter=','); Ltot = D[-1, 0] + (D[1, 0] - D[0, 0]); nb, _ = bins_of(D[:, 0], Ltot, a.bin)
        st = dict(base=a.base, profile=a.profile, bin=a.bin, f=[1.0] * nb, history=[])
        json.dump(st, open(a.state, 'w'), indent=1); print(f'init: {nb} bins of ~{Ltot / nb:.2f} m'); return
    st = json.load(open(a.state)); f = np.array(st['f'])
    if a.cmd == 'show':
        for i, x in enumerate(f): print(f'bin {i:2d}  f {x:.3f}')
        for h in st['history']: print(h['summary'])
        return
    D = np.loadtxt(st['base'], delimiter=','); S = D[:, 0]; Ltot = S[-1] + (S[1] - S[0]); nb, B = bins_of(S, Ltot, st['bin'])
    z = np.load(a.map); occ = z['occ']; ox, oy = z['origin']; R = float(z['res'])
    DF = ndimage.distance_transform_edt(~occ) * R - R / 2
    coll = np.zeros(nb); clr = np.full(nb, 9.0); dev = [[] for _ in range(nb)]; seen = np.zeros(nb, bool)
    for lf in a.logs:
        Lg = np.load(lf); t = Lg[:, 0]; keep = t >= a.skip_t
        idx = np.clip(Lg[:, 9].astype(int), 0, len(S) - 1); b = B[idx]; cc = Lg[:, 11]
        d = ndimage.map_coordinates(DF, [(Lg[:, 2] - oy) / R, (Lg[:, 1] - ox) / R], order=1, mode='nearest') - a.half_w
        for k in np.where(np.diff(cc) > 0)[0] + 1:   # 충돌: lookback 이전 pose의 구간으로 귀속(원인 구간)
            j = max(0, int(np.searchsorted(t, t[k] - a.lookback)) - 1)
            if keep[j]: coll[b[j]] += cc[k] - cc[k - 1]
        rec = np.zeros(len(t), bool)   # 충돌 후 2s(복귀 구간)는 여유/이탈 통계 제외
        for k in np.where(np.diff(cc) > 0)[0] + 1: rec |= (t >= t[k]) & (t < t[k] + 2.0)
        for k in np.where(keep & ~rec)[0]:
            clr[b[k]] = min(clr[b[k]], d[k]); dev[b[k]].append(abs(Lg[k, 10])); seen[b[k]] = True
    p95 = np.array([np.percentile(x, 95) if x else 0.0 for x in dev]); f0 = f.copy()
    for i in range(nb):
        if coll[i] > 0:
            f[i] -= min(a.down * coll[i], 2 * a.down); f[(i - 1) % nb] -= min(a.down * coll[i], 2 * a.down) / 2
        elif not seen[i]: continue
        elif clr[i] < a.c_lo: f[i] -= a.down / 3
        elif clr[i] > a.c_hi and p95[i] < a.e_hi: f[i] += a.up
    f = np.clip(0.5 * f + 0.25 * np.roll(f, 1) + 0.25 * np.roll(f, -1), a.fmin, a.fmax)   # 이웃 평활(급변 방지)
    summ = f'logs={len(a.logs)} coll={int(coll.sum())} bins up={int((f > f0 + 1e-4).sum())} down={int((f < f0 - 1e-4).sum())} f mean {f.mean():.3f} min {f.min():.3f} max {f.max():.3f}'
    print(summ)
    for i in range(nb):
        if abs(f[i] - f0[i]) > 1e-4 or coll[i]:
            print(f'  bin {i:2d} s {i * Ltot / nb:5.1f}-{(i + 1) * Ltot / nb:5.1f}: coll {int(coll[i])} clr {clr[i]:.3f} dev95 {p95[i]:.3f}  f {f0[i]:.3f}->{f[i]:.3f}')
    st['f'] = f.round(4).tolist(); st['history'].append(dict(logs=a.logs, summary=summ, coll=coll.tolist(), clr=np.where(clr > 8, None, clr.round(3)).tolist()))
    json.dump(st, open(a.state, 'w'), indent=1)
    if a.out: write_line(st, a.out)


if __name__ == '__main__':
    main()
