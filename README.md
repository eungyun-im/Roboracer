# roboracer_ws — AutoDRIVE RoboRacer Sim Racing 준비 작업공간

어디에 두어도 동작(모든 스크립트가 자기 위치 기준). 상세 실험 기록과 다음 계획은 **NOTES.md**.

## 새 컴퓨터 설치 (Ubuntu 22.04 + ROS 2 Humble 전제)
```bash
git clone https://github.com/eungyun-im/Roboracer.git && cd Roboracer   # 원하는 위치에
./setup.sh --sims        # venv + 파이썬 의존성 + devkit 빌드 + 시뮬(연습 Porto, IROS26 결승) 다운로드
source env.sh            # 매 터미널
```
setup.sh가 누락 시스템 패키지(apt)를 알려주면 직접 설치 후 다시 실행. 다른 시뮬: `bash tools/fetch_sims.sh icra26 cdctf25 explore`

## 실행
1. 브리지:  `ros2 launch autodrive_roboracer bringup_headless.launch.py`
2. 시뮬:   `cd sim/c_2026-iros/autodrive_simulator && ./"AutoDRIVE Simulator.x86_64"` → 왼쪽 위 메뉴 → 연결(Disconnected) 클릭
   (또는 `bash tools/sim_up.sh sim/c_2026-iros/autodrive_simulator` — xdotool로 자동 클릭, 화면 사용 중엔 쓰지 말 것)
3. 데이터 확인: `python3 tools/rate.py` (≈18 Hz)
4. 현재 최고 설정 주행(워밍업1+레이스10): `bash tools/run_combo_iros.sh`  → 레이스10 ≈ 78.9 s, 레이스 충돌 0 (이전 설정: `run_final_iros.sh` ≈ 79.9 s)
5. Porto(예선) 시험: `bash tools/run_porto.sh [line.csv] [laps]` — 재최적화 후보는 `results/porto_cand.json` 참고

브리지가 멈추면(0 Hz, 포트 4567 점유): 브리지 프로세스 `kill -9` 후 1번 재실행 → 시뮬이 자동 재연결.

## 대회 제출 (race/)
대회 모드(`--comp 1`)는 금지 토픽(/odom, /ips, /tf, lap_*, collision_count)을 구독하지 않고 reset 도 보내지 않으며, 랩 수/타임아웃으로 끝나지 않는다.
```bash
docker build -f race/Dockerfile -t <TEAM>/<IMAGE>:<TAG> .     # 베이스: autodrive_roboracer_api:2026-iros-compete (브리지 무수정)
bash race/run_local.sh [auto|porto|iros26|explore]              # 도커 없이 같은 방식으로 로컬 실행 → 시뮬에서 연결 클릭
bash race/audit_topics.sh                                       # (실행 중 다른 터미널) 우리 노드의 금지 토픽 사용 여부 점검
```
- 엔트리포인트 `race/autodrive_devkit.sh` = 공식 브리지 + `race/race_launch.py`. 컨테이너를 `--entrypoint /bin/bash` 로 띄웠다면 `/home/autodrive_devkit.sh` 를 직접 실행.
- `race_launch.py`: 첫 LiDAR 스캔을 각 트랙 맵의 출발 pose에서 레이캐스팅한 예상 거리와 비교해 트랙 판별(`race/tracks.json`), 위치추정+주행 노드 기동·감시(죽으면 재기동). `TRACK=` 로 강제 가능.
- 모르는 트랙이면 **explore**: `tools/explore.py`(FTG 탐색 주행 + `slam_lite.py` 온라인 SLAM, GT 없음) 한 바퀴 → `tools/auto_track.py` stage A(수 초: 맵·중심선·최소곡률 라인) → 위치추정/주행 인계 → stage B(최소시간 라인)가 나오면 주행 중 라인 교체(`racer --reload 1`).
- MPC 명령 지연은 측정 주기가 18 Hz에서 15% 이상 벗어날 때만 프레임 수 비례로 조정(`--mdelay_hz0 18`, 18 Hz에서는 기존과 동일).
- ROS/시뮬 없이 돌리는 시험(스텁 + 가상 LiDAR):
  `python3 race/test_comp_topics.py`(금지 토픽) · `python3 race/test_identify.py`(트랙 판별) · `python3 race/test_localizer_offline.py [iros26|porto]`(위치추정/인계) · `python3 race/test_explore_offline.py [map center]`(탐색→매핑→라인)

## 구조
| 경로 | 내용 |
|---|---|
| src/autodrive_devkit | 공식 devkit(브리지) — 수정 금지(대회 규칙) |
| tools/localizer.py | 위치추정: IMU yaw + 엔코더(+타이어관측기 --vobs) + LiDAR CSM (GT 미사용) |
| tools/racer.py, racer_core.py | 주행: MPC(지연/서보 모델, qy 속도스케줄), 속도 FF, 슬립 제어 옵션 |
| tools/raceline.py, mintime.py, profile_region.py | 최소곡률/최소시간 라인, 곡률연속 그립·속도연동 벽여유, 구간 속도 배율 |
| tools/build_track.py, map_track.sh | 매핑. ftg_record 기록(GT, 연습용) 또는 SLAM 기록(GT 없음: 궤적 기반 중심선 + 통로 마스크) |
| tools/slam_lite.py, explore.py | GT 없는 매핑: IMU yaw + 엔코더 + LiDAR 스캔-맵 정합, 루프 폐합 / 탐색 주행 노드 |
| tools/auto_track.py | 매핑 기록 → 맵(5cm·2.5cm) → 중심선 → 최소곡률/최소시간 라인 → 프로파일 (IROS26 수동 절차 자동화) |
| tools/scan_sim.py | 오프라인 시험용 가상 LiDAR/엔코더(맵 레이캐스팅) |
| tools/ilc_profile.py, ilc_iter.sh | 구간별 학습형 속도 보정(ILC): 연습 주행 로그로 구간 배율 자동 조정 → 새 라인. `bash tools/ilc_iter.sh 4` |
| tools/adsim.py | AutoDRIVE 물리 재현 오프라인 시뮬 — **미보정**(실제보다 충돌 많음), 로직 동작 확인용. bag으로 보정 필요 |
| racer --guard 1 | LiDAR 직접 안전층(실험적, 기본 꺼짐): 예측 차체가 벽과 겹치면 조향 최소 보정/감속. 로그에 개입률 출력 |
| race/ | 제출 패키지: Dockerfile, 엔트리포인트, race_launch.py, tracks.json, 시험/감사 스크립트 |
| tools/run_race_test.sh | 대회형 테스트(노드 선기동→리셋 즉시 출발), LOC_ARGS/ENC_SCALE/MAP/CENTER env |
| tools/loc_offline.py, bag2npz.py, true_dev.py, sweep2_analyze.py ... | 오프라인 분석 도구 |
| maps/ | 맵(*_map.npz), 중심선, 라인. 최종: iros26_combo.csv(결승), porto_mt_best.csv(예선), 계획용 2.5cm맵 r025/, Porto 후보 porto_cand/ |
| results/ | 최적 설정 JSON (iros26_combo.json, porto_mt_best.json, porto_cand.json …) |
| data/ | 매핑 원자료(*_ftg.npz), 분석용 bag 추출본(bag_*.npz) — **레포에 없음**(용량), 원본 PC에서 복사 필요. bags/, downloads/ 도 동일 |
| docs/ | 참고: 시뮬 소스 기반 차량 물리 정리 |
| third_party/f1tenth_gym | gym 보정용(선택) |

## 규칙 주의
런타임에 GT 토픽(/odom, /ips, /tf, lap/collision 등) 사용 금지. 제출 패키지는 `--comp 1` 로만 노드를 띄우므로 GT·lap·collision 토픽을 구독하지 않는다
(`race/test_comp_topics.py` 로 확인). run_race_test.sh 의 --gt_check 덤프와 lap 토픽 집계는 분석/테스트 전용(대회 모드에선 비활성).
