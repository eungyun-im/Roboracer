#!/bin/bash
# D-3 플레이북 1단계: 결승 트랙 공개 당일, 새 시뮬(트랙)에 연결한 상태에서 실행.
#   GT 매핑 주행(연습 단계라 허용, 대회 런타임 아님) → 맵(5cm/2.5cm)·중심선(궤적 기반) → 최소시간 라인(IROS26 검증 절차)
#   → 라인 검증 → race/tracks.json 등록 → 트랙 판별 시험
# usage: new_track.sh <name> [base=iros26] [ftg_speed=1.5]
# 이후: bash tools/run_track.sh <name>  →  bash tools/ilc_iter.sh <name> 4  →  bash race/preflight.sh
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
N=$1; BASE=${2:-iros26}; V=${3:-1.5}; [ -z "$N" ] && { echo "usage: new_track.sh <name> [base] [speed]"; exit 1; }
mkdir -p data logs maps
set -e
echo "[1/5] 매핑 주행 (FTG ${V} m/s, 2랩, GT 기록)"
python3 tools/steptest.py >/dev/null 2>&1 || true   # 리셋 겸 구동 확인
python3 tools/ftg_record.py data/${N}_ftg.npz $V 2 2>&1 | grep -v XMLPARSER
echo "[2/5] 맵/중심선/라인 생성 (stage A+B)"
TRAJ_CENTER=1 python3 tools/auto_track.py data/${N}_ftg.npz maps/${N} --stage AB --margin_add 0 | tee logs/new_track_${N}.log | grep -E "^\[auto\]|status|length"
echo "[3/5] 라인 검증"
python3 race/test_lines.py maps/${N}_line.csv
echo "[4/5] tracks.json 등록 (설정 기준: $BASE)"
python3 tools/track_add.py $N --pre maps/${N} --base $BASE | head -3
echo "[5/5] 트랙 판별 시험(첫 스캔으로 구분되는지)"
python3 race/test_identify.py || echo "!! 판별 실패 — 출발 위치가 다른 트랙과 너무 비슷함: TRACK=$N 로 강제 실행 필요"
echo "다음: bash tools/run_track.sh $N   (11랩 검증)  →  bash tools/ilc_iter.sh $N 4  →  bash race/preflight.sh"
