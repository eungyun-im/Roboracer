#!/bin/bash
# 현재 최고 설정(results/iros26_final.json)으로 IROS26 결승 트랙 대회형 주행 (시뮬=sim/c_2026-iros 연결 상태)
# usage: run_final_iros.sh [laps=11]
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
C=$(python3 -c "import json;c=json.load(open('results/iros26_final.json'))['cfg'];print(f\"--vscale 1.0 --steer_ay 8.5 --steer_slip {c['steer_slip']} --vlead 0.15 --gov_e0 {c['gov_e0']} --ctrl mpc --mdelay {c['mdelay']} --qpsi {c['qpsi']} --vff 0 --aff 0.22 --aff_neg 0 --qy 1 --rd 15 --qy_hi 3\")")
MAP=maps/iros26_map.npz CENTER=maps/iros26_center.csv ENC_SCALE=0.941 tools/run_race_test.sh maps/iros26_final.csv ${1:-11} $C --timeout 200
