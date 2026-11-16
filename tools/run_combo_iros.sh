#!/bin/bash
# 최고 조합(results/iros26_combo.json): 곡률상한조향+헤어핀그립8.1+제동최적슬립+타이어관측기+충돌복귀 안전장치
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
C=$(python3 -c "import json;c=json.load(open('results/iros26_combo.json'))['cfg'];print(f\"--vscale 1.0 --steer_ay 8.5 --steer_slip {c['steer_slip']} --vlead 0.15 --gov_e0 {c['gov_e0']} --ctrl mpc --mdelay {c['mdelay']} --qpsi {c['qpsi']} --vff 0 --aff 0.22 --aff_neg 0 --qy 1 --rd 15 --qy_hi 3 --afcap 2.0 --smode 2 --sbrk 0.15 --kthr 0.0396 --rec_vmax 4.0\")")
LOC_ARGS='--vobs 1 --tele_r 2.0 --tele_dpen 0.05 --rec_time 3.0' MAP=maps/iros26_map.npz CENTER=maps/iros26_center.csv ENC_SCALE=0.941 tools/run_race_test.sh maps/iros26_combo.csv ${1:-11} $C --timeout 200
