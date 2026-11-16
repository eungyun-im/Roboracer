#!/bin/bash
# Porto(예선, practice 빌드) 대회형 시험: race/tracks.json 의 porto 설정(맵/엔코더/racer 인자)으로 라인만 바꿔 주행
# usage: run_porto.sh [line.csv=tracks.json porto.line] [laps=11]   (테스트 모드: 리셋 출발 + 랩/충돌 집계)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
eval "$(python3 -c "
import json,shlex; t=json.load(open('race/tracks.json'))['tracks']['porto']
print('MAP='+t['map'],'CENTER='+t['center'],'ENC='+str(t['enc_scale']),'DEF_LINE='+t['line'],'LOCA='+shlex.quote(t['loc_args']),'RARGS='+shlex.quote(t['racer_args']))")"
LINE=${1:-$DEF_LINE}; LAPS=${2:-11}
echo "line=$LINE laps=$LAPS"
LOC_ARGS="$LOCA" MAP=$MAP CENTER=$CENTER ENC_SCALE=$ENC tools/run_race_test.sh $LINE $LAPS $RARGS --timeout 200
