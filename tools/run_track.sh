#!/bin/bash
# 아무 트랙이나 race/tracks.json 설정으로 대회형 시험(테스트 모드: 리셋 출발 + 랩/충돌 집계)
# usage: run_track.sh <track> [line.csv=설정 라인] [laps=11] [추가 racer 인자...]
#   예) run_track.sh iros26 '' 11 --smode 3 --spred 2 --sacc 0.10 --scap 1.0
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
TR=$1; L=$2; N=${3:-11}; shift 3 2>/dev/null || shift $#
eval "$(python3 -c "
import json,shlex,sys; t=json.load(open('race/tracks.json'))['tracks'].get('$TR')
if t is None: sys.exit('unknown track $TR')
print('MAP='+t['map'],'CENTER='+t['center'],'ENC='+str(t['enc_scale']),'DEF_LINE='+t['line'],'LOCA='+shlex.quote(t['loc_args']),'RARGS='+shlex.quote(t['racer_args']))")" || exit 1
LINE=${L:-$DEF_LINE}; echo "track=$TR line=$LINE laps=$N extra=$*"
LOC_ARGS="$LOCA" MAP=$MAP CENTER=$CENTER ENC_SCALE=$ENC tools/run_race_test.sh $LINE $N $RARGS --timeout 300 "$@"
