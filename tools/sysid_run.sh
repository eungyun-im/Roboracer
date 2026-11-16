#!/bin/bash
# 시스템 식별 데이터 수집(연습 주행 — GT 기록 허용, 대회 런타임 아님). IROS26 결승 빌드 시뮬 연결 상태에서 실행.
# 기록: GT odom(x,y,vx,vy,wz), IMU, 엔코더, 명령(throttle/steering_command), 실제 액추에이터(throttle/steering), 충돌/랩
# 주행: ① 현재 최고 설정 11랩  ② 코너 그립 스윕(lin_a0 9.2→9.6→10.0, 각 4랩: 한계/미끄러짐 데이터)  ③ 가속 개선(C2) 4랩
# 결과: data/sysid_<시각>.npz  → 레포에 올려주면 tools/adsim_fit.py 로 시뮬 보정
# usage: sysid_run.sh [track=iros26]   (약 10분)
cd "$(dirname "$(readlink -f "$0")")/.." && source env.sh
TR=${1:-iros26}; TS=$(date +%m%d_%H%M); B=bags/sysid_$TS; mkdir -p bags data maps/sysid logs
P=/autodrive/roboracer_1
ros2 bag record -o $B $P/odom $P/imu $P/left_encoder $P/right_encoder $P/throttle_command $P/steering_command \
    $P/throttle $P/steering $P/collision_count $P/lap_count /racer/pose /racer/odom > logs/sysid_bag.log 2>&1 &
BAG=$!; sleep 2
LINE=$(python3 -c "import json;print(json.load(open('race/tracks.json'))['tracks']['$TR']['line'])")
echo "[1/3] 현재 설정 11랩"; bash tools/run_track.sh $TR '' 11
echo "[2/3] 코너 그립 스윕"
for A in 9.2 9.6 10.0; do
    python3 tools/profile_region.py $LINE maps/sysid/sweep_a$A.csv --lin_a0 $A --lin_slope 2.5 --lin_min 6.5 --aacc 6.0 --abrk 8.5 --drag 0.273
    bash tools/run_track.sh $TR maps/sysid/sweep_a$A.csv 4
done
echo "[3/3] 가속 개선(C2) 4랩"
[ "$TR" = iros26 ] && bash tools/run_track.sh $TR maps/iros_cand/iros26_tc_a7.csv 4 --smode 3 --spred 2 --sacc 0.15 --scap 1.0
kill -INT $BAG; wait $BAG 2>/dev/null; sleep 1
python3 tools/bag2npz.py $B data/sysid_$TS.npz && echo "saved data/sysid_$TS.npz  ($(du -h data/sysid_$TS.npz | cut -f1)) → git add -f data/sysid_$TS.npz 후 push"
