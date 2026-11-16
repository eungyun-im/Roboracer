#!/bin/bash
# AutoDRIVE 기준 데이터: (alat tight smooth vscale)
cd "$(dirname "$(readlink -f "$0")")/.."
for cfg in "8 7 6 1.0" "9 7.5 15 1.0" "10 8 6 1.0" "8 7 6 0.7"; do set -- $cfg
  f=maps/porto_rl_a${1}_t${2}_s${3}.csv
  [ -f $f ] || python3 tools/raceline.py maps/porto_center.csv $f --alat $1 --alat_tight $2 --smooth $3 --aacc 4.5 --abrk 5 >/dev/null
  echo "CFG $cfg"; tools/run_loc_test.sh $f 5 --vscale $4 --out results/calib_refs.jsonl | grep -E "^lap"
done
