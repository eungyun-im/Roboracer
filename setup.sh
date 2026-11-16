#!/bin/bash
# 새 컴퓨터 1회 설치: (사전) Ubuntu 22.04 + ROS 2 Humble 설치되어 있어야 함
# usage: ./setup.sh            -> venv 생성 + 파이썬 의존성 + devkit(colcon) 빌드
#        ./setup.sh --sims     -> 위 + 시뮬레이터 빌드 다운로드(약 300MB, tools/fetch_sims.sh)
set -e
WS="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; cd "$WS"
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS 2 Humble 필요: /opt/ros/humble 없음"; exit 1; }
echo "[1/4] 시스템 패키지 확인 (없으면 아래 명령을 직접 실행: sudo 필요)"
MISSING=""
python3 -c "import venv, ensurepip" 2>/dev/null || MISSING="$MISSING python3-venv"
python3 -c "import Xlib" 2>/dev/null || MISSING="$MISSING python3-xlib"
python3 -c "import scipy, matplotlib, PIL" 2>/dev/null || MISSING="$MISSING python3-scipy python3-matplotlib python3-pil"
dpkg -s ros-humble-cv-bridge >/dev/null 2>&1 || MISSING="$MISSING ros-humble-cv-bridge"
dpkg -s ros-humble-tf-transformations >/dev/null 2>&1 || MISSING="$MISSING ros-humble-tf-transformations"
command -v xdotool >/dev/null || MISSING="$MISSING xdotool"
if [ -n "$MISSING" ]; then echo "  누락:$MISSING"; echo "  -> sudo apt install$MISSING"; else echo "  OK"; fi
python3 -c "import transforms3d" 2>/dev/null || echo "  (tf_transformations용) pip3 install --user transforms3d"
echo "[2/4] venv 생성(시스템 패키지 공유) + requirements 설치"
[ -d venv ] || python3 -m venv --system-site-packages venv
venv/bin/python -m pip install -q --upgrade pip
venv/bin/python -m pip install -q -r requirements.txt
echo "[3/4] devkit 빌드 (colcon)"
source /opt/ros/humble/setup.bash
mkdir -p logs; colcon build --symlink-install --base-paths src > logs/colcon_build.log 2>&1 || { echo "colcon 실패: logs/colcon_build.log"; exit 1; }
echo "[4/4] 시뮬레이터"
if [ "$1" = "--sims" ]; then bash tools/fetch_sims.sh; else echo "  (생략) 필요 시: bash tools/fetch_sims.sh"; fi
echo "완료. 매 터미널:  source $WS/env.sh"
