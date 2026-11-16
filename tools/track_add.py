#!/usr/bin/env python3
"""새 트랙을 race/tracks.json 에 등록(auto_track.py 출력 사용). 물리/제어 설정은 같은 빌드의 기존 트랙(--base)에서 복사.
usage: track_add.py <name> --pre maps/<name> [--base iros26] [--cfg race/tracks.json] [--line 다른라인.csv]
결승 트랙(compete 빌드)이면 --base iros26(새 물리), 연습 빌드면 --base porto."""
import argparse, json, os, shutil, time
ap = argparse.ArgumentParser(); ap.add_argument('name'); ap.add_argument('--pre', required=True); ap.add_argument('--base', default='iros26')
ap.add_argument('--cfg', default='race/tracks.json'); ap.add_argument('--line', default='')
a = ap.parse_args()
c = json.load(open(a.cfg)); b = c['tracks'][a.base]
t = dict(note=f'{time.strftime("%Y-%m-%d")} 등록(auto_track, 설정은 {a.base} 기준) — 실주행 검증 전',
         map=a.pre + '_map.npz', center=a.pre + '_center.csv', line=a.line or a.pre + '_line.csv', plan_map=a.pre + '_r025_map.npz',
         enc_scale=b['enc_scale'], loc_args=b['loc_args'], racer_args=b['racer_args'], profile=b.get('profile', ''))
for k in ('map', 'center', 'line', 'plan_map'):
    if not os.path.exists(t[k]): raise SystemExit(f'missing {k}: {t[k]}')
if os.path.exists(a.cfg): shutil.copy(a.cfg, a.cfg + '.bak')
c['tracks'][a.name] = t
json.dump(c, open(a.cfg, 'w'), indent=2, ensure_ascii=False); open(a.cfg, 'a').write('\n')
print(f'registered {a.name} in {a.cfg} (backup {a.cfg}.bak):'); print(json.dumps(t, indent=1, ensure_ascii=False))
