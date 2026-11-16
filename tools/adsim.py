#!/usr/bin/env python3
"""[미보정 — 수치로 결론 내지 말 것] AutoDRIVE RoboRacer 물리 재현 오프라인 시뮬(결승 빌드 기준).
현재 상태(2026-11-16): 실측 combo 설정(실제 레이스 충돌 0, 7.9s/랩)이 이 시뮬에선 랩당 1~3 충돌·11~12s.
  추종 오차(±7cm)는 실측과 비슷하나 헤어핀 출구 충돌·속도가 안 맞음. 실주행 bag(GT odom+명령)으로 동역학/충돌 박스 보정 필요.
  지금은 제어 로직이 돌아가는지(동작 원리) 확인용으로만 사용.
model='pheno'(기본): 실측 식별 기반 현상 모델 / model='phys': Unity 타이어 곡선 기반 동적 자전거 모델.
근거: docs/ref_abdullah_VEHICLE_MODEL.md(시뮬 소스) + NOTES 실측(결승 빌드).
- 스로틀 = 바퀴 표면속도 u = 25.25·θ·0.97 (θ=0이면 바퀴 잠금), 모터는 한계 없음 → 종력은 타이어가 결정
- 종슬립 S=(u-vx)/max(|vx|,4), 종마찰 Unity 곡선(0.15→0.72g, 0.25→0.464g) × k_mu
- 횡마찰: 실측 앞바퀴 μ(α) (0.6°:0.88, 1.75°:0.72, 3.5°:0.61, 5.7°↑:0.5), 결합은 마찰원으로 제한
- 동적 자전거 모델(m 3.906, CoM 뒤축 앞 0.155, L 0.324, h 0.06, Iz), 선형 공기저항 0.273·v, 각저항 0.1
- 조향 서보 3.2 rad/s(중앙쪽은 즉시: 결승 빌드 실측), 명령은 lockstep으로 delay 후 적용, 제어 18Hz
- 위치추정 노이즈(AR(1) 종/횡), 속도추정 노이즈, 벽 충돌 시 +1 및 약 1m 뒤 중심선으로 정지 상태 재생성
pose 기준점 = 뒤축 중심(우리 맵/라인과 동일). 차체 사각형: 앞 0.41, 뒤 -0.09, 반폭 0.135.
usage(모듈): run(track, line_csv, laps, ctrl_kw, sim_kw, guard_kw=None) -> dict
usage(CLI):  adsim.py --line maps/iros26_combo.csv --cfg iros26 [--laps 11]"""
import sys, os, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from scipy import ndimage
from racer_core import Controller, MPCController, Guard, MPC_DEFAULTS

G = 9.81; L = 0.324; LR = 0.155; LF = L - LR
SIM = dict(model='pheno', K=0.007, tau_w=0.04, a0=9.8, a_k0=0.6, a_slope=2.7, a_min=7.0, slide=0.75, a_tot=10.0,
           kv=4.0, a_acc=5.3, a_brk=7.4, m=3.906, Iz=0.10, h=0.06, drag=0.273, adrag=0.1, k_mu=1.1, lat_scale=1.0, ugain=25.25 * 0.97,
           srate=3.2, servo_asym=1, af_scale=1.0, ar_scale=1.0, delay=0.075, tdelay=None, hz=18.0, dt=0.002,
           n_lon=0.04, n_lat=0.015, n_rho=0.95, n_v=0.08, half_w=0.135, front=0.41, rear=-0.09,
           respawn_back=1.0, rec_time=3.0, seed=0, timeout=400.0)
# 횡마찰 μ(α[deg]) 실측(앞바퀴, NOTES 2026-11-15 dyn_ident) — 0~0.6°는 t(2-t) 상승
AL_PTS = np.array([0.6, 1.0, 1.75, 3.5, 5.7, 90.0]); MU_PTS = np.array([0.88, 0.86, 0.72, 0.61, 0.50, 0.50])


def mu_lat(a_deg, scale):
    a = abs(a_deg)
    if a < AL_PTS[0]:
        t = a / AL_PTS[0]; m = MU_PTS[0] * t * (2 - t)
    else:
        m = float(np.interp(a, AL_PTS, MU_PTS))
    return m * scale


def mu_lon(s):
    s = abs(s); XE, YE, XA, YA = 0.15, 0.72, 0.25, 0.464
    if s <= XE: t = s / XE; return YE * t * (2 - t)
    if s <= XA: t = (s - XE) / (XA - XE); return YE + (YA - YE) * (3 * t * t - 2 * t ** 3)
    return YA


def load_track(map_npz, center_csv):
    z = np.load(map_npz); occ = z['occ']; ox, oy = z['origin']; R = float(z['res'])
    DF = ndimage.distance_transform_edt(~occ) * R - R / 2
    C = np.loadtxt(center_csv, delimiter=',')[:, :2]
    st = z['start'] if 'start' in z.files and len(z['start']) >= 3 else None
    return dict(DF=DF, ox=float(ox), oy=float(oy), R=R, C=C, start=st)


def body_clear(T, x, y, th, S):
    fr, re, hw = S['front'], S['rear'], S['half_w']
    pts = np.array([[fr, hw], [fr, -hw], [re, hw], [re, -hw], [fr, 0], [(fr + re) / 2, hw], [(fr + re) / 2, -hw], [re, 0], [0.25, hw], [0.25, -hw]])
    c, s = np.cos(th), np.sin(th); W = np.c_[x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]]
    d = ndimage.map_coordinates(T['DF'], [(W[:, 1] - T['oy']) / T['R'], (W[:, 0] - T['ox']) / T['R']], order=1, mode='constant', cval=-1)
    return float(d.min())


class PhenoCar:
    """실측 식별 기반 현상 모델(NOTES): 명령 지연·서보(3.2rad/s, 중앙쪽 즉시) → 요레이트 목표 v·tanδ/(L(1+Kv²)),
    1차 지연 tau_w. 횡가속 한계 A(κ)=max(a_min, a0 - a_slope·max(κ-a_k0,0)) (코너 스윕: κ0.3-0.6 ~9.2 안전/10.2 충돌,
    κ0.6-0.9 8.2/9.0, 헤어핀 7.4/8.7) 초과 시 미끄러짐(횡가속 slide·A로 떨어짐 → 바깥으로 밀림).
    종방향: 바퀴속도 u 와 종슬립 마찰곡선(타이어 관측기와 동일) - drag·v, 마찰원 a_tot."""
    def __init__(s, S, x, y, th):
        s.S = S; s.set(x, y, th)
    def set(s, x, y, th):
        s.x, s.y, s.th = x, y, th; s.vx = 0.0; s.vy = 0.0; s.r = 0.0; s.d = 0.0; s.sliding = False; s.u = 0.0
    def pose(s): return s.x, s.y, s.th
    def step(s, dcmd, theta, h):
        S = s.S
        if S['servo_asym'] and abs(dcmd) <= abs(s.d) and np.sign(dcmd) == np.sign(s.d): s.d = dcmd
        else: s.d += float(np.clip(dcmd - s.d, -S['srate'] * h, S['srate'] * h))
        s.d = float(np.clip(s.d, -0.5236, 0.5236)); v = max(s.vx, 0.0)
        kc = np.tan(s.d) / (L * (1 + S['K'] * v * v)); wt = v * kc
        A = max(S['a_min'], S['a0'] - S['a_slope'] * max(abs(np.tan(s.d) / L) - S['a_k0'], 0.0))
        ay_t = v * wt
        s.sliding = abs(ay_t) > A
        if s.sliding: wt = np.sign(wt) * S['slide'] * A / max(v, 0.3)
        s.r += (wt - s.r) * min(h / S['tau_w'], 1.0)
        ay = v * s.r
        u = S['ugain'] * theta if theta > 0 else 0.0   # θ=0: 바퀴 잠금
        s.u = u
        Sx = (u - v) / max(v, 4.0)   # 종슬립 → 마찰곡선(tools/tire_obs.py와 동일, 실측 검증)
        ax = float(np.sign(Sx) * mu_lon(Sx) * S['k_mu'] * G) - S['drag'] * v
        amax_x = np.sqrt(max(S['a_tot'] ** 2 - ay ** 2, 1.0)); ax = float(np.clip(ax, -amax_x, amax_x))
        s.vx = max(v + ax * h, 0.0); s.vy = LR * s.r
        s.x += s.vx * np.cos(s.th) * h; s.y += s.vx * np.sin(s.th) * h; s.th += s.r * h


class Car:
    def __init__(s, S, x, y, th):
        s.S = S; s.set(x, y, th)
    def set(s, x, y, th):
        # 상태: CoM 위치/자세, 차체 속도(vx,vy), 요레이트 r, 실제 조향 d
        s.X = x + LR * np.cos(th); s.Y = y + LR * np.sin(th); s.th = th; s.vx = 0.0; s.vy = 0.0; s.r = 0.0; s.d = 0.0; s.ax = 0.0
    def pose(s):   # 뒤축 기준
        return s.X - LR * np.cos(s.th), s.Y - LR * np.sin(s.th), s.th
    def step(s, dcmd, theta, h):
        S = s.S
        # 조향 서보
        if S['servo_asym'] and abs(dcmd) <= abs(s.d) and np.sign(dcmd) == np.sign(s.d): s.d = dcmd
        else: s.d += float(np.clip(dcmd - s.d, -S['srate'] * h, S['srate'] * h))
        s.d = float(np.clip(s.d, -0.5236, 0.5236))
        u = S['ugain'] * theta; locked = theta <= 0.0
        m = S['m']; Nf = m * G * LR / L - m * s.ax * S['h'] / L; Nr = m * G * LF / L + m * s.ax * S['h'] / L
        Nf = max(Nf, 0.1); Nr = max(Nr, 0.1)
        cd, sd = np.cos(s.d), np.sin(s.d)
        # 바퀴 좌표 속도
        vfx = cd * s.vx + sd * (s.vy + LF * s.r); vfy = -sd * s.vx + cd * (s.vy + LF * s.r)
        vrx = s.vx; vry = s.vy - LR * s.r
        F = []
        for vwx, vwy, N, asc in ((vfx, vfy, Nf, S['af_scale']), (vrx, vry, Nr, S['ar_scale'])):
            uw = 0.0 if locked else u
            Sx = (uw - vwx) / max(abs(vwx), 4.0)
            fx = np.sign(Sx) * mu_lon(Sx) * S['k_mu'] * N
            a = np.degrees(np.arctan2(vwy, max(abs(vwx), 0.3)))
            fy = -np.sign(a) * mu_lat(a / asc, S['lat_scale']) * N   # asc>1: 코너링 강성 완화(실측 언더스티어/요 지연 보정용)
            fmax = max(mu_lon(0.15) * S['k_mu'], mu_lat(0.6, S['lat_scale'])) * N   # 마찰원
            n = np.hypot(fx, fy)
            if n > fmax: fx *= fmax / n; fy *= fmax / n
            F.append((fx, fy))
        (ffx, ffy), (frx, fry) = F
        Fx = cd * ffx - sd * ffy + frx; Fy = sd * ffx + cd * ffy + fry
        Mz = LF * (sd * ffx + cd * ffy) - LR * fry
        ax = Fx / m - S['drag'] * s.vx; ay = Fy / m - S['drag'] * s.vy
        s.ax = ax
        s.vx += (ax + s.vy * s.r) * h; s.vy += (ay - s.vx * s.r) * h
        s.r += (Mz / S['Iz'] - S['adrag'] * s.r) * h
        if s.vx < 0.05 and theta <= 0.0: s.vx = max(s.vx, 0.0)
        c, sn = np.cos(s.th), np.sin(s.th)
        s.X += (c * s.vx - sn * s.vy) * h; s.Y += (sn * s.vx + c * s.vy) * h; s.th += s.r * h


def run(T, line_csv, laps=11, ctrl=None, sim=None, guard=None, log=False, verbose=False, rec_vmax=4.0, kthr=0.0396, steer_cmd_scale=1.0):
    S = dict(SIM); S.update(sim or {}); rng = np.random.default_rng(S['seed'])
    ck = dict(ctrl or {}); mpc = ck.pop('ctrl', 'mpc') == 'mpc'
    K = MPCController(line_csv, **ck) if mpc else Controller(line_csv, **ck)
    Gd = Guard(**guard) if guard is not None else None
    if Gd is not None:
        from scan_sim import raycast, AMIN, AINC, LIDAR_X
        kb = np.arange(0, 1080, 4); angb = AMIN + kb * AINC
    C = T['C']; NC = len(C)
    if T['start'] is not None: x0, y0, th0 = map(float, T['start'][:3])
    else: tg = C[1] - C[0]; x0, y0, th0 = C[0, 0], C[0, 1], float(np.arctan2(tg[1], tg[0]))
    car = (PhenoCar if S['model'] == 'pheno' else Car)(S, x0, y0, th0)
    dtc = 1.0 / S['hz']; nsub = int(round(dtc / S['dt'])); h = S['dt']
    q = []; cmd = (0.0, 0.0); t = 0.0; el = ec = ev = 0.0
    def cidx(x, y, prev):
        w = np.arange(prev - 10, prev + 30) % NC; return int(w[np.argmin((C[w, 0] - x) ** 2 + (C[w, 1] - y) ** 2)])
    pi = int(np.argmin((C[:, 0] - x0) ** 2 + (C[:, 1] - y0) ** 2)); sline = pi
    laps_t = []; lap_t0 = 0.0; coll = 0; rec_until = -1; LOG = []; CEV = []; minclr = []; lapc = []
    lap_cnt = 0
    while t < S['timeout'] and len(laps_t) < laps:
        # --- 제어(18Hz): 노이즈 pose/속도 ---
        x, y, th = car.pose(); v = car.vx
        el = S['n_rho'] * el + np.sqrt(1 - S['n_rho'] ** 2) * rng.normal(0, S['n_lon'])
        ec = S['n_rho'] * ec + np.sqrt(1 - S['n_rho'] ** 2) * rng.normal(0, S['n_lat'])
        ev = S['n_rho'] * ev + np.sqrt(1 - S['n_rho'] ** 2) * rng.normal(0, S['n_v'])
        xo = x + el * np.cos(th) - ec * np.sin(th); yo = y + el * np.sin(th) + ec * np.cos(th); vm = max(v + ev, 0.0)
        delta, vref, e = K.step(xo, yo, th, vm, car.r, t=t) if mpc else K.step(xo, yo, th, vm, car.r)
        if t < rec_until: vref = min(vref, rec_vmax)
        if Gd is not None:
            r = raycast(T['DF'], T['ox'], T['oy'], T['R'], x, y, th)[kb]; ok = np.isfinite(r) & (r < 9.5)
            Gd.set_scan(np.c_[LIDAR_X + r[ok] * np.cos(angb[ok]), r[ok] * np.sin(angb[ok])], t)
            delta, vref, _ = Gd.filter(vm, delta, vref, t)
        theta = float(np.clip(vref * kthr, 0, 1))
        q.append((t + S['delay'], 'd', delta * steer_cmd_scale)); q.append((t + (S['delay'] if S['tdelay'] is None else S['tdelay']), 't', theta)); q.sort(key=lambda z: z[0])
        if log: LOG.append((t, x, y, th, v, car.vy, car.r, car.d, delta, vref, e, K.idx if K.idx is not None else -1, lap_cnt, theta, getattr(car, 'u', 0.0)))
        # --- 물리 ---
        crashed = False
        for _ in range(nsub):
            while q and q[0][0] <= t + 1e-9:   # 조향/스로틀 명령은 각자의 지연 후 반영
                _, kind, val = q.pop(0); cmd = (val, cmd[1]) if kind == 'd' else (cmd[0], val)
            car.step(cmd[0], cmd[1], h); t += h
        x, y, th = car.pose()
        ni = cidx(x, y, pi)
        if (pi - sline) % NC > NC // 2 and (ni - sline) % NC < NC // 2 and (ni - pi) % NC < NC // 2:
            if lap_cnt > 0 or t > 1.0: laps_t.append(t - lap_t0); lapc.append(coll)
            lap_t0 = t; lap_cnt += 1
            if verbose: print(f'lap {len(laps_t)} {laps_t[-1]:.3f} coll {coll}')
        pi = ni
        clr = body_clear(T, x, y, th, S); minclr.append(clr)
        if clr < 0:
            CEV.append((t, x, y, K.idx if K.idx is not None else -1)); coll += 1
            j = int((pi - int(S['respawn_back'] / 0.1)) % NC); tg = C[(j + 1) % NC] - C[j]
            car.set(C[j, 0], C[j, 1], float(np.arctan2(tg[1], tg[0]))); pi = j; K.reset(); q = []; cmd = (0.0, 0.0)
            rec_until = t + S['rec_time']
            if coll > 60: break
    res = dict(laps=laps_t, coll=coll, coll_per_lap=np.diff([0] + lapc).tolist(), total=float(sum(laps_t)),
               done=len(laps_t) >= laps, minclr=float(np.percentile(minclr, 1)) if minclr else 0)
    if Gd is not None: res['guard_act'] = Gd.n_act / max(Gd.n_calls, 1)
    if log: res['log'] = np.array(LOG); res['coll_ev'] = np.array(CEV).reshape(-1, 4)
    return res


CFG = {   # race/tracks.json 의 racer_args 와 같은 값(대회 설정)
    'iros26': dict(ctrl='mpc', vscale=1.0, steer_ay=8.5, steer_slip=2.5, vlead=0.15, gov_e0=0.15, mdelay=0.13, qpsi=1.0, vff=0, aff=0.22,
                   aff_neg=0, qy=1, rd=15, qy_hi=3, afcap=2.0, smode=2, sbrk=0.15),
    'iros26_final': dict(ctrl='mpc', vscale=1.0, steer_ay=8.5, steer_slip=2.5, vlead=0.15, gov_e0=0.15, mdelay=0.13, qpsi=1.0, vff=0, aff=0.22,
                         aff_neg=0, qy=1, rd=15, qy_hi=3),
}

if __name__ == '__main__':
    import argparse, json
    ap = argparse.ArgumentParser(); ap.add_argument('--map', default='maps/iros26_map.npz'); ap.add_argument('--center', default='maps/iros26_center.csv')
    ap.add_argument('--line', default='maps/iros26_combo.csv'); ap.add_argument('--cfg', default='iros26'); ap.add_argument('--laps', type=int, default=11)
    ap.add_argument('--sim', default='{}'); ap.add_argument('--sim_file', default=''); ap.add_argument('--ctrl', default='{}'); ap.add_argument('--guard', default=''); ap.add_argument('--seeds', default='0')
    a = ap.parse_args(); T = load_track(a.map, a.center)
    if a.sim_file: a.sim = json.dumps(dict(json.load(open(a.sim_file)).get('sim', {}), **json.loads(a.sim)))   # adsim_fit.py 결과 적용
    for sd in map(int, a.seeds.split(',')):
        r = run(T, a.line, a.laps, ctrl=dict(CFG[a.cfg], **json.loads(a.ctrl)), sim=dict(json.loads(a.sim), seed=sd),
                guard=(json.loads(a.guard) if a.guard else None))
        L_ = r['laps']; race = sum(L_[1:11]) if len(L_) >= 11 else float('nan')
        print(json.dumps(dict(seed=sd, race10=round(race, 2), laps=[round(v, 2) for v in L_], coll=r['coll'], per_lap=r['coll_per_lap'],
                              minclr=round(r['minclr'], 3), guard=round(r.get('guard_act', 0), 3))))
