#!/usr/bin/env python3
"""시뮬 보정 도구(tools/adsim_fit.py) 시험: 정답 파라미터로 adsim 주행 기록을 bag2npz 형식으로 합성 → 피팅 →
① 식별 가능한 파라미터(서보 속도, 스로틀 지연, 언더스티어 K, 그립 a0, k_mu)가 정답 근처인지
② 보정된 시뮬이 정답 시뮬과 같은 폐루프 주행 결과(랩타임, 충돌, 추종 오차)를 내는지.
조향 지연은 서보 피드백만으론 프레임 안에서 구분이 안 돼(18Hz) 요레이트와 함께 맞추고, 스로틀 지연은 차속과 함께 맞춤.
수 분 걸림(CI 제외). usage: python3 race/test_adsim_fit.py"""
import os, sys, json, subprocess, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE); sys.path.insert(0, os.path.join(WS, 'tools')); os.chdir(WS)
import adsim
TRUE = dict(model='pheno', delay=0.09, tdelay=0.15, srate=3.0, servo_asym=1, K=0.008, tau_w=0.05, a0=9.6, a_slope=2.0, slide=0.7, k_mu=1.1)
TOL = dict(tdelay=0.03, srate=0.5, K=0.003, a0=0.6, k_mu=0.1)

def main():
    tmp = tempfile.mkdtemp(prefix='fit_'); T = adsim.load_track('maps/r025/iros26_map.npz', 'maps/iros26_center.csv'); T['start'] = np.load('maps/iros26_map.npz')['start']
    subprocess.run([sys.executable, 'tools/profile_region.py', 'maps/iros26_combo.csv', f'{tmp}/agg.csv', '--lin_a0', '10.4', '--lin_slope', '1.0', '--lin_min', '7.5', '--aacc', '6.0', '--abrk', '8.5', '--drag', '0.273'], check=True, capture_output=True)
    S = dict(TRUE, front=0.30, rear=-0.03, half_w=0.115, n_lon=0.02, n_lat=0.01, n_v=0.05)
    rows = []; t_off = 0.0
    for line, laps in (('maps/iros26_combo.csv', 3), (f'{tmp}/agg.csv', 3)):
        r = adsim.run(T, line, laps, ctrl=dict(adsim.CFG['iros26']), sim=S, log=True); Lg = r['log'].copy(); Lg[:, 0] += t_off; t_off = Lg[-1, 0] + 1.0; rows.append(Lg)
    Lg = np.vstack(rows); t = Lg[:, 0]
    enc = np.cumsum(np.r_[0, np.diff(t)] * Lg[:, 14]) / 0.0581
    z = dict(odom=np.c_[t, t, Lg[:, 1], Lg[:, 2], Lg[:, 4], Lg[:, 5], Lg[:, 6]], steering_command=np.c_[t, Lg[:, 8] / 0.5236], steering=np.c_[t, Lg[:, 7] / 0.5236],
             throttle_command=np.c_[t, Lg[:, 13]], left_encoder=np.c_[t, t, enc, enc * 0], right_encoder=np.c_[t, t, enc, enc * 0])
    np.savez_compressed(f'{tmp}/sysid.npz', **z)
    subprocess.run([sys.executable, 'tools/adsim_fit.py', f'{tmp}/sysid.npz', f'{tmp}/fit.json'], check=True)
    fit = json.load(open(f'{tmp}/fit.json'))['sim']; ok = True
    for k, tol in TOL.items():
        e = abs(fit[k] - TRUE[k]); good = e <= tol; ok &= good
        print(f'  {k:8s} true {TRUE[k]:<7} fit {fit[k]:<7} {"OK" if good else "MISS"}')
    # ② 폐루프 등가성: 같은 제어 설정으로 정답/보정 시뮬 주행 비교(노이즈 시드 동일)
    base = dict(front=0.30, rear=-0.03, half_w=0.115)
    def closed(sim):
        L_ = []; C_ = 0; Dv = []
        for sd in (0, 1, 2, 3):
            r = adsim.run(T, 'maps/iros26_combo.csv', 4, ctrl=dict(adsim.CFG['iros26']), sim=dict(base, seed=sd, **sim), log=True)
            L_ += [x for x in r['laps'][1:] if x > 2]; C_ += r['coll']; Dv.append(np.percentile(np.abs(r['log'][:, 10]), 95))
        return np.median(L_), C_, float(np.mean(Dv))
    tr = closed(TRUE); ft = closed({k: v for k, v in fit.items()})
    print(f'  closed loop  true: lap {tr[0]:.2f} coll {tr[1]} dev95 {tr[2]:.3f} | fitted: lap {ft[0]:.2f} coll {ft[1]} dev95 {ft[2]:.3f}')
    # 이 가상 세계는 충돌이 잦아 변동이 큼(정답 시뮬도 시드만 바꾸면 충돌 9↔14) → 시드 4개, 여유 있는 기준
    eq = abs(tr[0] - ft[0]) < 0.8 and 0.5 <= (ft[1] + 1) / (tr[1] + 1) <= 2.0 and abs(tr[2] - ft[2]) < 0.08; ok &= eq
    print('  closed-loop equivalence', 'OK' if eq else 'MISS')
    print('OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
