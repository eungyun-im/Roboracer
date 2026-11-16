# roboracer_ws — AutoDRIVE RoboRacer Sim Racing 준비 작업공간

어디에 두어도 동작(모든 스크립트가 자기 위치 기준). 상세 실험 기록과 다음 계획은 **NOTES.md**.

## 새 컴퓨터 설치 (Ubuntu 22.04 + ROS 2 Humble 전제)
```bash
tar xzf roboracer_ws_bundle_*.tar.gz   # 원하는 위치에
cd roboracer_ws
./setup.sh --sims        # venv + 파이썬 의존성 + devkit 빌드 + 시뮬(연습 Porto, IROS26 결승) 다운로드
source env.sh            # 매 터미널
```
setup.sh가 누락 시스템 패키지(apt)를 알려주면 직접 설치 후 다시 실행. 다른 시뮬: `bash tools/fetch_sims.sh icra26 cdctf25 explore`

## 실행
1. 브리지:  `ros2 launch autodrive_roboracer bringup_headless.launch.py`
2. 시뮬:   `cd sim/c_2026-iros/autodrive_simulator && ./"AutoDRIVE Simulator.x86_64"` → 왼쪽 위 메뉴 → 연결(Disconnected) 클릭
   (또는 `bash tools/sim_up.sh sim/c_2026-iros/autodrive_simulator` — xdotool로 자동 클릭, 화면 사용 중엔 쓰지 말 것)
3. 데이터 확인: `python3 tools/rate.py` (≈18 Hz)
4. 현재 최고 설정 주행(워밍업1+레이스10): `bash tools/run_final_iros.sh`  → 레이스10 ≈ 79.9 s, 충돌 0

브리지가 멈추면(0 Hz, 포트 4567 점유): 브리지 프로세스 `kill -9` 후 1번 재실행 → 시뮬이 자동 재연결.

## 구조
| 경로 | 내용 |
|---|---|
| src/autodrive_devkit | 공식 devkit(브리지) — 수정 금지(대회 규칙) |
| tools/localizer.py | 위치추정: IMU yaw + 엔코더(+타이어관측기 --vobs) + LiDAR CSM (GT 미사용) |
| tools/racer.py, racer_core.py | 주행: MPC(지연/서보 모델, qy 속도스케줄), 속도 FF, 슬립 제어 옵션 |
| tools/raceline.py, mintime.py, profile_region.py | 최소곡률/최소시간 라인, 곡률연속 그립·속도연동 벽여유, 구간 속도 배율 |
| tools/build_track.py, map_track.sh | 매핑(현재 GT 사용 — 연습용) |
| tools/run_race_test.sh | 대회형 테스트(노드 선기동→리셋 즉시 출발), LOC_ARGS/ENC_SCALE/MAP/CENTER env |
| tools/loc_offline.py, bag2npz.py, true_dev.py, sweep2_analyze.py ... | 오프라인 분석 도구 |
| maps/ | 맵(*_map.npz), 중심선, 라인. 최종: iros26_final.csv, 계획용 2.5cm맵 r025/ |
| results/ | 최적 설정 JSON (iros26_final.json, porto_mt_best.json …) |
| data/ | 매핑 원자료(*_ftg.npz), 분석용 bag 추출본(bag_*.npz) |
| docs/ | 참고: 시뮬 소스 기반 차량 물리 정리 |
| third_party/f1tenth_gym | gym 보정용(선택) |

## 규칙 주의
런타임에 GT 토픽(/odom, /ips, /tf, lap/collision 등) 사용 금지. 우리 스택은 위치추정 모드(--pose loc)에서 GT 미사용
(run_race_test의 --gt_check 덤프와 lap 토픽 집계는 분석/테스트 전용 — 제출 패키지에선 제거 필요).
