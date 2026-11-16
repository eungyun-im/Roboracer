#!/usr/bin/env python3
"""미지 트랙 자동 처리: 매핑 기록(rec.npz) → 맵/중심선 → 레이싱라인 → 속도 프로파일.
IROS26에서 수동으로 찾은 절차(results/iros26_final.json, iros26_combo.json line_gen)를 그대로 자동화.
  stage A (빠름, 수 초): build_track(5cm 위치추정용, 2.5cm 계획용) → 최소곡률 라인(여유 0.35, 고무줄 정제)
                         → 곡률연속 그립 프로파일 → <pre>_line.csv, <pre>_ready.json
  stage B (느림, 수십 초~): 최소시간 라인(mintime, 속도연동 벽여유, 2.5cm 맵) → κ 재계산 → 프로파일
                         → 검사(벽 여유, 예상 랩 단축) 통과 시 <pre>_line.csv 를 원자적으로 교체(racer --reload 1이 감지)
usage: auto_track.py <rec.npz> <out_prefix> [--stage A|B|AB] [--margin_add 0.025]"""
import argparse, os, sys, json, subprocess, time, shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable


def sh(args, env=None, timeout=None):
    t0 = time.time(); e = dict(os.environ, **(env or {}))
    r = subprocess.run([PY] + args, env=e, capture_output=True, text=True, timeout=timeout)
    out = (r.stdout + r.stderr).strip()
    print(f'$ {" ".join(os.path.basename(a) if i == 0 else a for i, a in enumerate(args))}  ({time.time() - t0:.1f}s)\n  ' + out.replace('\n', '\n  '), flush=True)
    if r.returncode != 0: raise RuntimeError(f'failed: {args[0]}')
    return out


def est_lap(line):
    D = np.loadtxt(line, delimiter=','); ds = D[1, 0] - D[0, 0]
    return float(np.sum(ds / np.maximum(D[:, 5], 0.1)))


def wall_min(line, map_npz):
    from scipy import ndimage
    z = np.load(map_npz); occ = z['occ']; ox, oy = z['origin']; res = float(z['res'])
    DF = ndimage.distance_transform_edt(~occ) * res - res / 2
    P = np.loadtxt(line, delimiter=',')[:, 1:3]
    return float(ndimage.map_coordinates(DF, [(P[:, 1] - oy) / res, (P[:, 0] - ox) / res], order=1, mode='nearest').min())


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('rec'); ap.add_argument('pre')
    ap.add_argument('--stage', default='AB'); ap.add_argument('--margin_add', type=float, default=0.025)
    ap.add_argument('--grip', type=float, default=9.2)       # 곡률연속 그립 lin_a0 (결승 빌드 실측)
    ap.add_argument('--grip_slope', type=float, default=2.5)
    ap.add_argument('--mt_timeout', type=float, default=600)
    a = ap.parse_args(); pre = a.pre; T = lambda f: os.path.join(HERE, f)
    ma = a.margin_add
    prof = ['--lin_a0', str(a.grip), '--lin_slope', str(a.grip_slope), '--lin_min', '6.5', '--aacc', '6.0', '--abrk', '8.5', '--drag', '0.273']
    line = pre + '_line.csv'
    if 'A' in a.stage:
        sh([T('build_track.py'), a.rec, pre], env={'MAP_RES': '0.05'})
        sh([T('build_track.py'), a.rec, pre + '_r025'], env={'MAP_RES': '0.025'})
        # 기준 라인: 최소곡률 + 고무줄 정제(맵 거리장으로 벽 여유 직접 제약) — iros26_mt_best line_gen과 같은 설정
        sh([T('raceline.py'), pre + '_center.csv', pre + '_ref.csv', '--alat', '10.5', '--alat_tight', '6.5', '--smooth', '6',
            '--margin', f'{0.35 + ma:.3f}', '--band_map', pre + '_map.npz'])
        sh([T('profile_region.py'), pre + '_ref.csv', pre + '_lineA.csv'] + prof)
        wm = wall_min(pre + '_lineA.csv', pre + '_r025_map.npz')
        if wm < 0.2: raise SystemExit(f'stage A line too close to wall ({wm:.3f} m) — map/centerline 확인 필요')
        shutil.copy(pre + '_lineA.csv', line + '.tmp'); os.replace(line + '.tmp', line)
        json.dump(dict(map=pre + '_map.npz', center=pre + '_center.csv', line=line, plan_map=pre + '_r025_map.npz',
                       stage='A', est_lap=est_lap(line), wall_min=wm), open(pre + '_ready.json', 'w'), indent=1)
        print(f'[auto] stage A ready: est lap {est_lap(line):.2f}s, wall min {wm:.3f} m', flush=True)
    if 'B' in a.stage:
        # 최소시간 라인: 여유 0.175(+ma) + 속도연동(0.015/(m/s), 4m/s 초과) 최대 0.22(+ma), 곡률상한 2.5, 곡률연속 그립
        try:
            sh([T('mintime.py'), pre + '_ref.csv', pre + '_r025_map.npz', pre + '_mt.csv', '--step', '0.1', '--wdk', '0.3',
                '--margin', f'{0.175 + ma:.3f}', '--mg_vk', '0.015', '--mg_max', f'{0.22 + ma:.3f}', '--mg_v0', '4',
                '--vref_line', pre + '_lineA.csv', '--kmax', '2.5', '--lin_a0', str(a.grip)], timeout=a.mt_timeout)
            sh([T('raceline.py'), pre + '_center.csv', pre + '_mtk.csv', '--from_line', pre + '_mt.csv', '--resmooth', '0'])
            sh([T('profile_region.py'), pre + '_mtk.csv', pre + '_lineB.csv'] + prof)
        except Exception as e:
            print(f'[auto] stage B failed ({e}) — stage A line 유지', flush=True); return
        eA, eB = est_lap(pre + '_lineA.csv'), est_lap(pre + '_lineB.csv'); wm = wall_min(pre + '_lineB.csv', pre + '_r025_map.npz')
        ok = wm >= 0.175 + ma - 0.03 and eB < eA - 0.02
        print(f'[auto] stage B: est {eA:.2f} -> {eB:.2f}s, wall min {wm:.3f} m -> {"adopt" if ok else "reject"}', flush=True)
        if ok:
            shutil.copy(pre + '_lineB.csv', line + '.tmp'); os.replace(line + '.tmp', line)
            r = json.load(open(pre + '_ready.json')) if os.path.exists(pre + '_ready.json') else {}
            r.update(stage='B', est_lap=eB, wall_min=wm); json.dump(r, open(pre + '_ready.json', 'w'), indent=1)


if __name__ == '__main__':
    main()
