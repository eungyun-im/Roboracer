#!/bin/bash
# usage: sim_headless.sh <sim_dir> : 헤드리스(-batchmode -nographics) 실행, CLI ip/port로 자동연결 확인
cd "$(dirname "$(readlink -f "$0")")/.."; source env.sh
SIMDIR=$(realpath "$1")
kill $(pgrep -f "^./AutoDRIVE Simulator") 2>/dev/null; sleep 2
N0=$(grep -c Connected logs/bridge.log)
(cd "$SIMDIR" && setsid nohup "./AutoDRIVE Simulator.x86_64" -batchmode -nographics -ip 127.0.0.1 -port 4567 > "$WS"/logs/sim_hl.log 2>&1 &)
for i in $(seq 1 30); do sleep 1; [ "$(grep -c Connected logs/bridge.log)" -gt "$N0" ] && { echo "headless connected after ${i}s"; exit 0; }; done
echo "headless: no connection"; exit 2
