#!/usr/bin/env python3
"""D-3 플레이북 리허설(ROS/시뮬 없이): 기존 맵에서 GT 매핑 기록(ftg_record 형식)을 합성 → new_track.sh 와 같은 단계
(auto_track A+B, TRAJ_CENTER=1) → 임시 tracks.json 에 등록(track_add) → 라인 검증 → 위치추정 시험. 소요 시간과 예상 랩 출력.
usage: python3 race/test_new_track.py [map=maps/iros26_map.npz] [center=maps/iros26_center.csv] [ref_est=7.67]"""
import os, sys, json, shutil, subprocess, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE); os.chdir(WS); PY = sys.executable

def sh(args, env=None):
    r = subprocess.run([PY] + args, env=dict(os.environ, **(env or {})), capture_output=True, text=True)
    if r.returncode: print(r.stdout[-2000:], r.stderr[-2000:]); raise SystemExit(f'FAIL: {args[0]}')
    return r.stdout

def main():
    mp = sys.argv[1] if len(sys.argv) > 1 else 'maps/iros26_map.npz'; ce = sys.argv[2] if len(sys.argv) > 2 else 'maps/iros26_center.csv'
    ref = float(sys.argv[3]) if len(sys.argv) > 3 else 7.67
    tmp = tempfile.mkdtemp(prefix='newtrack_'); t0 = time.time()
    sh(['tools/scan_sim.py', mp, ce, f'{tmp}/sens.npz', '--v', '1.5', '--laps', '2.0', '--wobble', '0.1'])
    d = np.load(f'{tmp}/sens.npz'); P = np.c_[d['gx'], d['gy']]
    dist = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]; L = np.sum(np.hypot(*np.diff(np.loadtxt(ce, delimiter=',')[:, :2], axis=0).T))
    lap = (dist / (L * 0.995)).astype(int)   # ftg_record 의 lap 카운터 흉내(한 바퀴마다 +1)
    np.savez_compressed(f'{tmp}/rehearsal_ftg.npz', x=d['gx'], y=d['gy'], yaw=d['gyaw'], ranges=d['ranges'], coll=np.zeros(len(P), int), lap=lap)
    t1 = time.time()
    out = sh(['tools/auto_track.py', f'{tmp}/rehearsal_ftg.npz', f'{tmp}/rehearsal', '--stage', 'AB', '--margin_add', '0'], env={'TRAJ_CENTER': '1'})
    t2 = time.time()
    cfg = f'{tmp}/tracks.json'; shutil.copy('race/tracks.json', cfg)
    sh(['tools/track_add.py', 'rehearsal', '--pre', f'{tmp}/rehearsal', '--base', 'iros26', '--cfg', cfg])
    lt = sh(['race/test_lines.py', f'{tmp}/rehearsal_line.csv'], env={'TRACKS_CFG': cfg})
    loc = sh(['race/test_localizer_offline.py', 'rehearsal'], env={'TRACKS_CFG': cfg, 'TMPDIR': tmp})
    r = json.load(open(f'{tmp}/rehearsal_ready.json'))
    print([l for l in out.splitlines() if l.startswith('[auto]')])
    print(lt.splitlines()[1][-80:]); print([l for l in loc.splitlines() if l.startswith('[')])
    print(f'pipeline {t2 - t1:.1f}s (total incl. synthesis {time.time() - t0:.1f}s) | est lap {r["est_lap"]:.2f}s vs reference {ref:.2f}s | stage {r["stage"]}')
    ok = r['stage'] == 'B' and r['est_lap'] < ref + 0.15 and 'OK' in lt
    print('OK' if ok else 'FAIL', tmp); sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
