#!/bin/bash
# IROS26 결승 빌드: 현재 최고 조합(run_combo_iros.sh) + 가속 트랙션 제어(smode 3) 시험
# 기본 = 명령 이력 기반 예측(spred 2: 가속 트랙션 + 최적슬립 제동 모두 예측 차속 기준)
# 1D 종방향 시뮬(tools/long_sim.py, 실측 랩 ±0.1s 재현) 예측: 랩당 -0.16~-0.20s, 코너 그립 한계 시간 감소(10개 조건 모두)
# usage: run_tc_iros.sh [laps=11] [sacc=0.10] [scap=1.0] [spred=2]   (예전 단순판: run_tc_iros.sh 11 0.15 0.5 0)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
C=$(python3 -c "import json;print(json.load(open('race/tracks.json'))['tracks']['iros26']['racer_args'])")
C=$(echo "$C" | sed 's/--smode 2/--smode 3/')
LOC_ARGS='--vobs 1 --tele_r 2.0 --tele_dpen 0.05 --rec_time 3.0' MAP=maps/iros26_map.npz CENTER=maps/iros26_center.csv ENC_SCALE=0.941 \
    tools/run_race_test.sh maps/iros26_combo.csv ${1:-11} $C --sacc ${2:-0.10} --apred 6.0 --scap ${3:-1.0} --spred ${4:-2} --timeout 200
