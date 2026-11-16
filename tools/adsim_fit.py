#!/usr/bin/env python3
"""시뮬 보정(시스템 식별): 실주행 기록(tools/sysid_run.sh → bag2npz 형식 npz)에서 tools/adsim.py 현상 모델 파라미터 추정.
단계: ① 조향 서보(명령→실제 조향: 지연·속도제한·중앙쪽 즉시 여부) ② 스로틀→바퀴속도(지연·이득, 엔코더)
      ③ 종방향 타이어(k_mu: 바퀴속도→차속, 슬립 마찰곡선+drag) ④ 횡방향(언더스티어 K, 요 지연 tau_w, 곡률별 그립 a0/a_slope, 미끄러짐 slide)
출력 JSON: {"sim": {adsim SIM 키: 값}, "quality": {...}} → adsim.py --sim_file 로 사용.
usage: adsim_fit.py <sysid.npz> <out.json> [--r_eff 0.0581]"""
import sys, json, argparse
import numpy as np
from scipy.optimize import least_squares

G = 9.81; L = 0.324; SMAX = 0.5236


def mu_lon(s):
    s = np.abs(s); XE, YE, XA, YA = 0.15, 0.72, 0.25, 0.464
    t1 = s / XE; t2 = np.clip((s - XE) / (XA - XE), 0, 1)
    return np.where(s <= XE, YE * t1 * (2 - t1), YE + (YA - YE) * (3 * t2 ** 2 - 2 * t2 ** 3))


def trim_rmse(e, q=95):
    """상위 (100-q)% 오차 제외 RMS — 충돌 재생성(명령 초기화·순간이동) 같은 튀는 구간에 강건"""
    e = np.abs(np.asarray(e)); e = e[e <= np.percentile(e, q)] if len(e) > 20 else e
    return float(np.sqrt(np.mean(e ** 2)))


def tied_range(vals, errs, tol):
    """오차가 최소값+tol 이내인 후보 범위(18Hz 샘플에선 한 프레임 안 지연들이 구분 안 됨)"""
    m = min(errs); return [v for v, e in zip(vals, errs) if e <= m + tol]


def servo(tc, cmd, tq, delay, rate, asym, dt=0.002):
    """명령 계단열(tc,cmd)을 delay 후 속도제한(asym: 중앙쪽 즉시)으로 추종한 실제 조향을 tq 시각에서 샘플"""
    t = np.arange(tq[0] - 0.5, tq[-1] + dt, dt); idx = np.searchsorted(tc, t - delay, side='right') - 1
    c = np.where(idx >= 0, cmd[np.clip(idx, 0, None)], 0.0); a = np.zeros_like(t); x = 0.0; dm = rate * dt
    for i in range(len(t)):
        ci = c[i]
        if asym and abs(ci) <= abs(x) and ci * x >= 0: x = ci
        else: x += min(max(ci - x, -dm), dm)
        a[i] = x
    return np.interp(tq, t, a)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('npz'); ap.add_argument('out'); ap.add_argument('--r_eff', type=float, default=0.0581)
    ap.add_argument('--win', type=float, default=0.6); ap.add_argument('--steer_fb', default='auto')   # 실제 조향 피드백 단위: auto|norm(±1)|rad
    a = ap.parse_args(); D = np.load(a.npz); Q = {}; out = {}
    od = D['odom']; tmap = np.polyfit(od[:, 0], od[:, 1], 1)   # 수신시각 → 시뮬 시각
    T = lambda tr: np.polyval(tmap, tr)
    to = od[:, 1]; vx = od[:, 4]; vy = od[:, 5]; wz = od[:, 6]
    # ① 조향 서보
    sc = D['steering_command']; sf = D['steering']; tc = T(sc[:, 0]); cmd = sc[:, 1] * SMAX; tf = T(sf[:, 0])
    mode = a.steer_fb
    if mode == 'auto':   # 피드백 vs 정규화 명령의 기울기: ~1 → 정규화(±1), ~0.52 → 라디안
        ci = np.interp(tf, tc, sc[:, 1]); mm = np.abs(ci) > 0.05
        slope = float(np.sum(sf[mm, 1] * ci[mm]) / np.sum(ci[mm] ** 2)) if mm.any() else 1.0
        mode = 'rad' if slope < 0.76 else 'norm'; print(f'[0] steering feedback units: {mode} (slope vs normalized command {slope:.2f})')
    fb = sf[:, 1] * (SMAX if mode == 'norm' else 1.0)
    m = (tf > tc[0] + 1) & (tf < tc[-1]); tf, fb = tf[m], fb[m]
    best = None
    for asym in (0, 1):
        for d in np.arange(0.0, 0.26, 0.01):
            for r in (1.5, 2.0, 2.5, 3.0, 3.2, 3.5, 4.0, 5.0, 8.0):
                e = trim_rmse(servo(tc, cmd, tf, d, r, asym) - fb)
                if best is None or e < best[0]: best = (e, d, r, asym)
    e, d, r, asym = best
    for dd in np.arange(max(d - 0.01, 0), d + 0.0101, 0.002):   # 미세 조정
        for rr in np.arange(max(r - 0.5, 0.5), r + 0.51, 0.1):
            ee = trim_rmse(servo(tc, cmd, tf, dd, rr, asym) - fb)
            if ee < e: e, d, r = ee, dd, rr
    dgrid = np.round(np.arange(0.0, 0.25, 0.005), 3); cand = []
    for asm in (0, 1):
        errs = [trim_rmse(servo(tc, cmd, tf, dd, r, asm) - fb) for dd in dgrid]
        cand += [(dd, asm) for dd in tied_range(dgrid, errs, max(1e-4, 0.05 * min(errs) + np.radians(0.05)))]
    out.update(srate=round(float(r), 2)); Q['servo_rmse_deg'] = round(float(np.degrees(e)), 3)
    print(f'[1] servo: rate {r:.2f} rad/s, delay/asym candidates (indistinguishable at frame rate): {len(cand)} '
          f'({min(c[0] for c in cand):.3f}..{max(c[0] for c in cand):.3f}s)  rmse {np.degrees(e):.2f} deg')
    # ② 스로틀 → 바퀴속도(엔코더 평균, 시뮬 시각)
    th = D['throttle_command']; tth = T(th[:, 0]); el = D['left_encoder']; er = D['right_encoder']
    te0 = el[:, 1]; enc = 0.5 * (el[:, 2] + np.interp(te0, er[:, 1], er[:, 2]))
    u = np.diff(enc) / np.maximum(np.diff(te0), 1e-3) * a.r_eff; te = 0.5 * (te0[1:] + te0[:-1])   # 후방차분 = 직전 프레임 평균 바퀴속도(구간 중앙 시각)
    ok = (te > tth[0] + 1) & (te < tth[-1]) & (np.abs(u) < 30)
    tg = np.round(np.arange(0.0, 0.31, 0.005), 3); te_err = []; gains = []
    for dd in tg:
        idx = np.searchsorted(tth, te[ok] - dd, side='right') - 1; thv = th[np.clip(idx, 0, None), 1]
        mm = thv > 0.02; g = float(np.median(u[ok][mm] / thv[mm])) if mm.any() else 25.0
        te_err.append(trim_rmse(g * thv[mm] - u[ok][mm])); gains.append(g)
    tcand = tied_range(tg, te_err, max(1e-3, 0.05 * min(te_err))); ug = gains[int(np.argmin(te_err))]
    out.update(ugain=round(float(ug), 3)); Q['wheel_rmse'] = round(float(min(te_err)), 3)
    print(f'[2] throttle->wheel: gain {ug:.2f}, delay candidates {min(tcand):.3f}..{max(tcand):.3f}s  rmse {min(te_err):.3f} m/s')
    # ③ 종방향: 측정 바퀴속도 u(t) 입력 → 차속 적분(창 단위) vs odom vx
    W = max(5, int(a.win * 18)); starts = np.arange(18, len(to) - W - 1, W)
    if len(starts) > 150: starts = starts[np.linspace(0, len(starts) - 1, 150).astype(int)]
    NS = 6
    def lon_cost(tdl):
        def res_f(p):
            res = []
            for s0 in starts:
                v = vx[s0]; pr = []
                for k in range(s0, s0 + W):
                    h = (to[k + 1] - to[k]) / NS
                    for j in range(NS):
                        tt = to[k] + j * h; i = np.searchsorted(tth, tt - tdl, side='right') - 1; thv = th[max(i, 0), 1]
                        uu = ug * thv if thv > 0 else 0.0
                        S = (uu - v) / max(v, 4.0); v = max(v + (G * np.sign(S) * mu_lon(S) * p[0] - 0.273 * v) * h, 0.0)
                    pr.append(v)
                res.append(np.array(pr) - vx[s0 + 1:s0 + W + 1])
            return np.concatenate(res)
        return least_squares(res_f, [1.1], bounds=([0.5], [2.0]), loss='soft_l1', f_scale=0.2)
    lb = None
    for tdl in sorted(set(np.round(np.arange(min(tcand) - 0.03, max(tcand) + 0.031, 0.01), 3))):
        if tdl < 0: continue
        rl = lon_cost(tdl); c = trim_rmse(rl.fun)
        if lb is None or c < lb[0]: lb = (c, tdl, rl.x[0])
    out.update(tdelay=round(float(lb[1]), 3), k_mu=round(float(lb[2]), 3)); Q['lon_rmse'] = round(lb[0], 3)
    print(f'[3] longitudinal (joint): throttle delay {lb[1]:.3f}s k_mu {lb[2]:.3f}  rmse {lb[0]:.3f} m/s')
    # ④ 횡방향(현상 모델): 실제 조향(피드백) + vx → 요레이트, 창 단위 적분 vs odom wz
    sel = [s0 for s0 in starts if vx[s0] > 1.0]
    tfine = np.concatenate([to[k] + (to[k + 1] - to[k]) * np.arange(NS) / NS for s0 in sel for k in range(s0, s0 + W)])
    def lat_fit(dfine):
        def lat_res(p):
            K, tau, a0, sl, slide = p; res = []; n = 0
            for s0 in sel:
                r = wz[s0]; pr = []
                for k in range(s0, s0 + W):
                    h = (to[k + 1] - to[k]) / NS; v = max(vx[k], 0.1)
                    for j in range(NS):
                        dd = dfine[n]; n += 1
                        wt = v * np.tan(dd) / (L * (1 + K * v * v)); A = max(7.0, a0 - sl * max(abs(np.tan(dd) / L) - 0.6, 0.0))
                        if abs(v * wt) > A: wt = np.sign(wt) * slide * A / v
                        r += (wt - r) * min(h / max(tau, 1e-3), 1.0)
                    pr.append(r)
                res.append(np.array(pr) - wz[s0 + 1:s0 + W + 1])
            return np.concatenate(res)
        return least_squares(lat_res, [0.005, 0.04, 9.8, 2.7, 0.75], bounds=([0.0, 0.003, 7.0, 0.0, 0.3], [0.03, 0.2, 13.0, 8.0, 1.0]), loss='soft_l1', f_scale=0.1)
    best_l = None; dc = sorted(set(cand))
    if len(dc) > 12: dc = [dc[i] for i in np.linspace(0, len(dc) - 1, 12).astype(int)]
    for dd, asm in dc:
        rr_ = lat_fit(servo(tc, cmd, tfine, dd, r, asm)); c = trim_rmse(rr_.fun)
        if best_l is None or c < best_l[0]: best_l = (c, dd, asm, rr_)
    _, dsel, asel, rr = best_l; out.update(delay=round(float(dsel), 3), servo_asym=int(asel))
    print(f'[4] steering delay (joint with yaw) {dsel:.3f}s asym {asel}')
    K, tau, a0, sl, slide = rr.x
    out.update(K=round(float(K), 4), tau_w=round(float(tau), 3), a0=round(float(a0), 2), a_slope=round(float(sl), 2), slide=round(float(slide), 2))
    Q['lat_rmse_rad_s'] = round(trim_rmse(rr.fun), 3); Q['lat_r2'] = round(float(1 - np.var(rr.fun) / np.var(wz[[k for s0 in sel for k in range(s0 + 1, s0 + W + 1)]])), 3)
    ay = vx * wz; Q['ay_p99_measured'] = round(float(np.percentile(np.abs(ay[vx > 1]), 99)), 2)
    print(f'[4] lateral: K {K:.4f} tau_w {tau:.3f} a0 {a0:.2f} a_slope {sl:.2f} slide {slide:.2f}  rmse {Q["lat_rmse_rad_s"]} rad/s R2 {Q["lat_r2"]}')
    json.dump(dict(sim=dict(out, model='pheno'), quality=Q, source=a.npz), open(a.out, 'w'), indent=1)
    print('saved', a.out)


if __name__ == '__main__':
    main()
