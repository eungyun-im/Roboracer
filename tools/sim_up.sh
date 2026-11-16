#!/bin/bash
# usage: sim_up.sh <sim_dir(…/autodrive_simulator)>  : 기존 시뮬 종료 → 실행 → 메뉴 열고 연결 버튼 클릭 → 데이터 확인
cd "$(dirname "$(readlink -f "$0")")/.."; source env.sh
SIMDIR=$(realpath "$1")
pgrep -f "bringup_headless|autodrive_bridge" >/dev/null || { (setsid nohup ros2 launch autodrive_roboracer bringup_headless.launch.py > logs/bridge.log 2>&1 &); sleep 4; }
kill $(pgrep -f "^./AutoDRIVE Simulator") 2>/dev/null; sleep 2
chmod +x "$SIMDIR/AutoDRIVE Simulator.x86_64"
(cd "$SIMDIR" && setsid nohup "./AutoDRIVE Simulator.x86_64" -ip 127.0.0.1 -port 4567 -screen-fullscreen 0 -screen-width 1920 -screen-height 1080 > "$WS"/logs/sim.log 2>&1 &)
W=""; for i in $(seq 1 60); do W=$(xdotool search --class "AutoDRIVE Simulator.x86_64" 2>/dev/null | tail -1); [ -n "$W" ] && break; sleep 1; done
[ -z "$W" ] && { echo "sim window not found"; exit 1; }
sleep 8
N0=$(grep -c Connected logs/bridge.log)
xdotool windowactivate --sync $W 2>/dev/null; xdotool mousemove --window $W 76 44 click 1; sleep 1.5
xdotool mousemove --window $W 75 409 click 1; sleep 3
N1=$(grep -c Connected logs/bridge.log)
if [ "$N1" -gt "$N0" ]; then echo "connected ($SIMDIR)"; else echo "connect FAILED"; exit 2; fi
