#!/bin/bash
# usage: map_track.sh <name>  (시뮬이 해당 트랙으로 연결된 상태) : FTG 매핑 주행 → 맵/중심선 → 기본 레이싱라인
cd "$(dirname "$(readlink -f "$0")")/.."; source env.sh; N=$1
python3 tools/steptest.py >/dev/null 2>&1   # 리셋 겸 구동 확인
python3 tools/ftg_record.py data/${N}_ftg.npz 1.5 2 2>&1 | grep -v XMLPARSER
python3 tools/build_track.py data/${N}_ftg.npz maps/${N} && \
python3 tools/raceline.py maps/${N}_center.csv maps/${N}_best.csv --alat 8 --alat_tight 7 --smooth 6 --aacc 4.5 --abrk 5
