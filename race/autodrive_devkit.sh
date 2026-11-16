#!/bin/bash
# 대회 엔트리포인트: 공식 브리지(headless) + 우리 스택(race/race_launch.py)
#  - TRACK=auto(기본) | porto | iros26 | explore   (auto: 첫 스캔으로 트랙 판별, 모르는 트랙이면 explore)
#  - 컨테이너를 --entrypoint /bin/bash 로 띄운 경우에도 이 스크립트를 직접 실행하면 됨: /home/autodrive_devkit.sh
#  - 로그: <racing>/logs/race_*.log
source /opt/ros/humble/setup.bash
DEVKIT=${DEVKIT:-/home/autodrive_devkit}
[ -f "$DEVKIT/install/setup.bash" ] && source "$DEVKIT/install/setup.bash"
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
RACING=${RACING:-$DEVKIT/racing}
mkdir -p "$RACING/logs"
rm -f "$RACING/logs/race_state.json"   # 새 레이스: 트랙 판별부터(실행기 재기동 시에만 이 파일로 이어감)

ros2 launch autodrive_roboracer bringup_headless.launch.py > "$RACING/logs/race_bridge.log" 2>&1 &
BRIDGE=$!
python3 "$RACING/race/race_launch.py" &
RACE=$!
cleanup() { kill $RACE 2>/dev/null; wait $RACE 2>/dev/null; kill $BRIDGE 2>/dev/null; pkill -f autodrive_bridge 2>/dev/null; exit 0; }
trap cleanup SIGINT SIGTERM

# 브리지가 죽으면 재기동(시뮬은 자동 재연결). 레이스 실행기가 죽으면 재기동(노드들은 위치추정으로 복귀)
while true; do
    if ! kill -0 $BRIDGE 2>/dev/null; then
        echo "[entry] bridge exited -> restart"; pkill -9 -f autodrive_bridge 2>/dev/null; sleep 0.5
        ros2 launch autodrive_roboracer bringup_headless.launch.py >> "$RACING/logs/race_bridge.log" 2>&1 &
        BRIDGE=$!
    fi
    if ! kill -0 $RACE 2>/dev/null; then
        echo "[entry] race_launch exited -> restart"
        python3 "$RACING/race/race_launch.py" &
        RACE=$!
    fi
    sleep 1
done
