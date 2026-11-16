#!/bin/bash
# 실행 중 감사: 우리 노드(racer/localizer/explore/race_launch)가 금지 토픽을 구독하거나 reset을 발행하는지 확인.
# 대회 스택 실행 중 다른 터미널에서: source env.sh && bash race/audit_topics.sh
RESTRICTED='/autodrive/roboracer_1/(odom|ips|lap_count|lap_time|last_lap_time|best_lap_time|collision_count)$|^ *(/tf|/tf_static)[: ]'
FAIL=0
for N in /racer /localizer /explore /race_launch; do
    INFO=$(ros2 node info $N 2>/dev/null) || continue
    SUBS=$(echo "$INFO" | sed -n '/Subscribers:/,/Publishers:/p' | grep -v -E 'Subscribers:|Publishers:')
    PUBS=$(echo "$INFO" | sed -n '/Publishers:/,/Service Servers:/p' | grep -v -E 'Publishers:|Service Servers:')
    echo "== $N"; echo "$SUBS" | sed 's/^/  sub /'; echo "$PUBS" | sed 's/^/  pub /'
    if echo "$SUBS" | sed 's/:.*//' | grep -qE "$RESTRICTED"; then echo "  !! 금지 토픽 구독"; FAIL=1; fi
    if echo "$PUBS" | grep -q '/autodrive/reset_command'; then echo "  !! reset_command 발행"; FAIL=1; fi
done
[ $FAIL = 0 ] && echo "OK: 금지 토픽 사용 없음" || { echo "FAIL"; exit 1; }
