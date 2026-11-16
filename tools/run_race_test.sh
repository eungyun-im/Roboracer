#!/bin/bash
# 대회형 출발 테스트: 위치추정+주행 노드를 먼저 띄워 대기 -> 리셋 순간 출발 (랩타이머=리셋부터)
# usage: run_race_test.sh <line.csv> <laps> [racer args...]   (MAP/CENTER env)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
LINE=$1; LAPS=$2; shift 2
python3 tools/localizer.py --map ${MAP:-maps/porto_map.npz} --center ${CENTER:-maps/porto_center.csv} --enc_scale ${ENC_SCALE:-0.97} ${LOC_ARGS} --gt_check 1 --dump data/locdump.npy > logs/loc.log 2>&1 &
LP=$!
python3 tools/racer.py --line $LINE --pose loc --reset 0 --arm_on_reset 1 --laps $LAPS "$@" > logs/racer_run.log 2>&1 &
RP=$!
sleep 3   # 노드 준비 대기 (랩타이머 시작 전)
python3 -c "
import time,rclpy; from std_msgs.msg import Bool; rclpy.init(); n=rclpy.create_node('rst'); p=n.create_publisher(Bool,'/autodrive/reset_command',10); time.sleep(0.3)
for v,d in ((True,0.15),(False,0.15)):
    t0=time.time()
    while time.time()-t0<d: p.publish(Bool(data=v)); time.sleep(0.01)
" 2>/dev/null
wait $RP; kill $LP 2>/dev/null; wait $LP 2>/dev/null
grep -E "^lap|TIMEOUT|RESULT|armed" logs/racer_run.log
