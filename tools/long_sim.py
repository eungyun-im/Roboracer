#!/usr/bin/env python3
"""종방향(1차원) 랩 시뮬: 라인 위를 정확히 따라간다고 가정하고 속도만 시뮬 — 가감속 제어 법칙 비교용.
물리(시뮬 소스 + 실측 검증된 타이어 관측기와 동일): 스로틀 = 바퀴 표면속도 u = ugain·θ (θ=0이면 잠금),
  종슬립 S=(u-v)/max(v,4), 가속도 = g·sign(S)·μ(S)·k_mu·√(1-(ay/ay_max)²) - drag·v  (μ: Unity 곡선 0.15→0.72, 0.25→0.464)
명령은 cdelay 후 바퀴에 반영, 속도 측정은 mdelay 지연 + 노이즈. 속도 명령 로직은 racer_core.Controller 그대로.
usage: long_sim.py --line maps/iros26_combo.csv [--smode 2] [--cdelay 0.11] [--laps 3] [기타 Controller 인자 key=val]"""
import sys, os, argparse, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from racer_core import Controller

G = 9.81


def mu_lon(s):
    s = abs(s); XE, YE, XA, YA = 0.15, 0.72, 0.25, 0.464
    if s <= XE: t = s / XE; return YE * t * (2 - t)
    if s <= XA: t = (s - XE) / (XA - XE); return YE + (YA - YE) * (3 * t * t - 2 * t ** 3)
    return YA


def run(line, ctrl, laps=3, cdelay=0.11, mdelay=0.055, hz=18.0, dt=0.002, k_mu=1.1, drag=0.273, ugain=25.25 * 0.97,
        kthr=0.0396, ay_max=10.5, v_noise=0.05, seed=0, log=False, a0=10.0, a_k0=0.6, a_slope=2.7, a_min=7.5):
    D = np.loadtxt(line, delimiter=','); S, X, Y, PSI, K = D[:, 0], D[:, 1], D[:, 2], D[:, 3], D[:, 4]; ds = S[1] - S[0]; Ltot = S[-1] + ds
    C = Controller(line, **ctrl); rng = np.random.default_rng(seed)
    s = 0.0; v = 0.0; t = 0.0; u = 0.0; q = []; vh = []; laps_t = []; lap0 = 0.0; LOG = []; theta = 0.0
    nsub = int(round(1 / hz / dt)); travelled = 0.0; over = 0.0; over_n = 0
    # 실측 횡그립 한계(코너 스윕에서 충돌이 난 값 근처): 넘으면 미끄러져 속도가 한계로 깎임 + 위험 지표 누적
    ALIM = np.maximum(a_min, a0 - a_slope * np.maximum(np.abs(K) - a_k0, 0)); VLIM = np.sqrt(ALIM / np.maximum(np.abs(K), 1e-4))
    while len(laps_t) < laps and t < 300:
        i = int(s / ds) % len(S)
        vm = vh[0][1] if vh else 0.0
        vm = max(vm + rng.normal(0, v_noise), 0.0)
        _, vref, _ = C.step(X[i], Y[i], PSI[i], vm, vm * K[i])
        theta = float(np.clip(vref * kthr, 0, 1)); q.append((t + cdelay, theta))
        if log: LOG.append((t, s, v, vref, u, V_plan(D, s)))
        for _ in range(nsub):
            while q and q[0][0] <= t + 1e-9: theta_a = q.pop(0)[1]; u = ugain * theta_a if theta_a > 0 else 0.0
            k = K[int(s / ds) % len(S)]; ay = v * v * abs(k)
            Sx = (u - v) / max(v, 4.0); fr = np.sqrt(max(0.0, 1 - min(ay / ay_max, 1.0) ** 2))
            a = G * np.sign(Sx) * mu_lon(Sx) * k_mu * fr - drag * v
            v = max(v + a * dt, 0.0)
            vl = VLIM[int(s / ds) % len(S)]
            if v > vl: over += (v - vl) * dt; over_n += 1; v = vl
            s += v * dt; travelled += v * dt; t += dt
            vh.append((t, v));
            while vh and vh[0][0] < t - mdelay: vh.pop(0)
            if s >= Ltot:
                s -= Ltot; laps_t.append(t - lap0); lap0 = t
    out = dict(laps=laps_t, over=over / max(t, 1e-6) * 100, over_time=over_n * dt / max(t, 1e-6))
    if log: out['log'] = np.array(LOG)
    return out


def V_plan(D, s):
    return float(np.interp(s, D[:, 0], D[:, 5]))


CTRL = dict(vscale=1.0, steer_ay=8.5, steer_slip=2.5, vlead=0.15, gov_e0=0.15, vff=0, aff=0.22, aff_neg=0, smode=2, sbrk=0.15, lat=0.0)

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--line', default='maps/iros26_combo.csv'); ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--cdelay', type=float, default=0.11); ap.add_argument('--ctrl', default='{}'); ap.add_argument('--seeds', default='0,1,2')
    a = ap.parse_args(); c = dict(CTRL, **json.loads(a.ctrl))
    fl = []; ov = []
    for sd in map(int, a.seeds.split(',')):
        r = run(a.line, c, a.laps, cdelay=a.cdelay, seed=sd); fl += r['laps'][1:]; ov.append(r['over_time'])
    print(json.dumps(dict(flying_mean=round(float(np.mean(fl)), 3), grip_limit_time_frac=round(float(np.mean(ov)), 4), laps=[round(x, 3) for x in fl])))
