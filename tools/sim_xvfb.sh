#!/bin/bash
# usage: sim_xvfb.sh <sim_dir> [WxH]
# 가상 디스플레이(Xvfb :9, 인증 없음)에서 시뮬 실행 -> 메뉴 열기/연결 버튼 클릭(xdotool, DISPLAY=:9)
cd "$(dirname "$(readlink -f "$0")")/.."; source env.sh
SIMDIR=$(realpath "$1"); RES=${2:-1280x720}
kill $(pgrep -f "^./AutoDRIVE Simulator") 2>/dev/null
kill $(pgrep -f "^Xvfb :9") 2>/dev/null; sleep 2
(setsid nohup Xvfb :9 -screen 0 ${RES}x24 -ac > logs/xvfb.log 2>&1 &); sleep 2
N0=$(grep -c Connected logs/bridge.log)
(cd "$SIMDIR" && DISPLAY=:9 setsid nohup "./AutoDRIVE Simulator.x86_64" -ip 127.0.0.1 -port 4567 -screen-fullscreen 0 -screen-width ${RES%x*} -screen-height ${RES#*x} > "$WS"/logs/sim_xvfb.log 2>&1 &)
W=""; for i in $(seq 1 60); do W=$(DISPLAY=:9 xdotool search --class "AutoDRIVE Simulator.x86_64" 2>/dev/null | tail -1); [ -n "$W" ] && break; sleep 1; done
[ -z "$W" ] && { echo "no window"; exit 1; }
sleep 10
echo "window $W: $(DISPLAY=:9 xdotool getwindowgeometry $W | tr '\n' ' ')"
