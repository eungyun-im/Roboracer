#!/bin/bash
# 구간별 학습형 속도 보정 반복(연습 주행): 11랩 주행(로그) → ilc_profile update → 다음 라인. 트랙 설정은 race/tracks.json
# usage: ilc_iter.sh [track=iros26] [iterations=4] [추가 racer 인자...]   (결과 라인: maps/ilc/<track>_itN.csv)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
TR=${1:-iros26}; N=${2:-4}; shift 2 2>/dev/null || shift $#
ST=data/ilc_$TR.json; mkdir -p data maps/ilc logs
eval "$(python3 -c "
import json,shlex; t=json.load(open('race/tracks.json'))['tracks']['$TR']
print('BASE='+t['line'],'PMAP='+t.get('plan_map',t['map']),'PROF='+shlex.quote(t.get('profile','')))")"
[ -f $ST ] || python3 tools/ilc_profile.py init $ST --base $BASE --profile "$PROF"
python3 tools/ilc_profile.py update $ST --map $PMAP --out maps/ilc/${TR}_it0.csv >/dev/null
for i in $(seq 1 $N); do
    LOG=data/ilc_${TR}_run$i.npy
    bash tools/run_track.sh $TR maps/ilc/${TR}_it$((i-1)).csv 11 --log $LOG "$@"
    python3 tools/ilc_profile.py update $ST $LOG --map $PMAP --skip_t 10 --out maps/ilc/${TR}_it$i.csv
done
python3 tools/ilc_profile.py show $ST | tail -n $((N+1))
