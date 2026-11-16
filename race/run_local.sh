#!/bin/bash
# 로컬(도커 없이) 대회 모드 시험: 공식 브리지 + race_launch.py 를 대회 엔트리포인트와 같은 방식으로 실행.
# usage: race/run_local.sh [auto|porto|iros26|explore]   → 이후 시뮬에서 연결(Disconnected) 클릭
# 주의: 대회 모드는 리셋/랩 집계를 하지 않음. 랩타임은 시뮬 HUD 또는 별도 터미널의 ros2 topic echo(분석용)로 확인.
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
export TRACK=${1:-auto}
DEVKIT="$WS" RACING="$WS" exec bash race/autodrive_devkit.sh
