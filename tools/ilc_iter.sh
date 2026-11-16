#!/bin/bash
# 구간별 학습형 속도 보정 반복(연습 주행, 결승 빌드 IROS26 기준): 11랩 주행(로그) → ilc_profile update → 다음 라인
# usage: ilc_iter.sh <iterations=4> [state=data/ilc_iros.json]   (시뮬 연결 상태, 결과 라인: maps/ilc/iros_itN.csv)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
N=${1:-4}; ST=${2:-data/ilc_iros.json}; mkdir -p data maps/ilc logs
[ -f $ST ] || python3 tools/ilc_profile.py init $ST --base maps/iros26_final.csv
python3 tools/ilc_profile.py update $ST --map maps/r025/iros26_map.npz --out maps/ilc/iros_it0.csv >/dev/null   # 로그 없이 현재 배율로 라인 생성
for i in $(seq 1 $N); do
    LINE=maps/ilc/iros_it$((i-1)).csv; LOG=data/ilc_run$i.npy
    C=$(python3 -c "import json;print(' '.join(json.load(open('race/tracks.json'))['tracks']['iros26']['racer_args'].split()))")
    LOC_ARGS='--vobs 1 --tele_r 2.0 --tele_dpen 0.05 --rec_time 3.0' MAP=maps/iros26_map.npz CENTER=maps/iros26_center.csv ENC_SCALE=0.941 \
        tools/run_race_test.sh $LINE 11 $C --timeout 200 --log $LOG
    python3 tools/ilc_profile.py update $ST $LOG --map maps/r025/iros26_map.npz --skip_t 10 --out maps/ilc/iros_it$i.csv
done
python3 tools/ilc_profile.py show $ST | tail -n $((N+1))
