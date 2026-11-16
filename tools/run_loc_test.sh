#!/bin/bash
# usage: run_loc_test.sh <line.csv> <laps> [racer args...]
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
LINE=$1; LAPS=$2; shift 2
python3 tools/reset_car.py 2>/dev/null
python3 tools/localizer.py --map ${MAP:-maps/porto_map.npz} --center ${CENTER:-maps/porto_center.csv} --gt_check 1 --dump data/locdump.npy > logs/loc.log 2>&1 &
LP=$!; sleep 1
python3 tools/racer.py --line $LINE --pose loc --reset 0 --laps $LAPS "$@" 2>&1 | grep -E "^lap|TIMEOUT|RESULT"
kill $LP 2>/dev/null; wait $LP 2>/dev/null
grep "loc err" logs/loc.log | tail -4
