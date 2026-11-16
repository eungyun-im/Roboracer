#!/bin/bash
# D-1 제출 전 최종 점검: 설정 파일 존재, 오프라인 시험 전부, (가능하면) 도커 빌드.  usage: race/preflight.sh [image_tag]
cd "$(dirname "$(readlink -f "$0")")/.."; [ -f env.sh ] && source env.sh 2>/dev/null
FAIL=0; ok(){ echo "  OK  $1"; }; ng(){ echo "  FAIL $1"; FAIL=1; }
echo "[1] tracks.json 파일 확인"
python3 - <<'PY' || FAIL=1
import json,os,sys; c=json.load(open('race/tracks.json')); bad=0
for n,t in c['tracks'].items():
    for k in ('map','center','line','plan_map'):
        if k in t and not os.path.exists(t[k]): print(f'  FAIL {n}.{k} missing: {t[k]}'); bad=1
    print(f'  {n:10s} line={t["line"]}  args: {" ".join(t["racer_args"].split()[:4])} ...')
sys.exit(bad)
PY
echo "[2] 오프라인 시험"
for T in "race/test_comp_topics.py" "race/test_identify.py" "race/test_lines.py" "race/test_long_sim.py"; do
    python3 $T >/tmp/preflight_$$.log 2>&1 && ok "$T" || { ng "$T"; tail -5 /tmp/preflight_$$.log; }
done
for TR in $(python3 -c "import json;print(' '.join(json.load(open('race/tracks.json'))['tracks']))"); do
    python3 race/test_localizer_offline.py $TR >/tmp/preflight_$$.log 2>&1 && ok "localizer $TR" || { ng "localizer $TR"; tail -3 /tmp/preflight_$$.log; }
done
rm -f /tmp/preflight_$$.log
echo "[3] 도커"
if command -v docker >/dev/null; then
    docker build -q -f race/Dockerfile -t ${1:-roboracer:preflight} . && ok "docker build ${1:-roboracer:preflight}" || ng "docker build"
else echo "  skip (docker 없음)"; fi
[ $FAIL = 0 ] && echo "PREFLIGHT OK" || { echo "PREFLIGHT FAIL"; exit 1; }
