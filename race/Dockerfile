####################################################
#  RoboRacer Sim Racing 제출 이미지
#  빌드(작업공간 루트에서):
#    docker build -f race/Dockerfile -t <TEAM_USERNAME>/<IMAGE_NAME>:<TAG> .
#  베이스 = 공식 devkit(ROS 2 API) 이미지 — 브리지는 수정하지 않고 그대로 사용(대회 규칙)
####################################################
ARG BASE=autodriveecosystem/autodrive_roboracer_api:2026-iros-compete
FROM ${BASE}

# 우리 스택 의존성 (numpy는 베이스의 1.22.2 고정 유지: 브리지 호환)
RUN pip3 install --no-cache-dir \
        numpy==1.22.2 \
        scipy==1.10.1 \
        osqp==1.1.3 \
        casadi==3.8.2 \
        scikit-image==0.21.0 \
        matplotlib==3.7.5

# 레이싱 스택(위치추정/주행/미지 트랙 매핑) + 맵/라인/설정
WORKDIR /home/autodrive_devkit/racing
COPY tools   ./tools
COPY maps    ./maps
COPY results ./results
COPY race    ./race
RUN mkdir -p logs maps/auto \
    && python3 -m py_compile tools/racer.py tools/racer_core.py tools/localizer.py tools/explore.py \
        tools/slam_lite.py tools/auto_track.py tools/build_track.py race/race_launch.py

# 엔트리포인트: 공식 브리지 + race_launch.py (트랙 자동 판별)
COPY race/autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_devkit.sh
WORKDIR /home/autodrive_devkit
EXPOSE 4567
ENTRYPOINT ["/home/autodrive_devkit.sh"]
