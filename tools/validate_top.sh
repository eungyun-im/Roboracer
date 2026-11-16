#!/bin/bash
# 상위 설정을 AutoDRIVE Porto 10랩(자체 위치추정)으로 검증
cd "$(dirname "$(readlink -f "$0")")/.."; source env.sh
python3 - <<'PY' > results/top3.txt
import json
rows=json.load(open('results/search/rows.json')); ok=sorted([r for r in rows if r['coll']==0 and r['done']],key=lambda r:r['norm'])[:3]
for r in ok: c=r['cfg']; print(r['id'],c['margin'],c['smooth'],c['alat'],c['alat_tight'],c['ld0'],c['ldk'],c['steer_ay'],c['vlead'])
PY
while read id mg sm al at ld0 ldk say vl; do
  f=maps/porto_top_$id.csv
  python3 tools/raceline.py maps/porto_center.csv $f --from_line maps/search/porto_g_m${mg}_s${sm}.csv --alat $al --alat_tight $at --aacc 4.5 --abrk 5 >/dev/null
  echo "TOP id=$id margin=$mg smooth=$sm alat=$al tight=$at ld0=$ld0 ldk=$ldk steer_ay=$say"
  tools/run_loc_test.sh $f 10 --vscale 1.0 --ld0 $ld0 --ldk $ldk --steer_ay $say --vlead $vl --out results/top3_porto.jsonl | grep -E "^lap|RESULT"
done < results/top3.txt
