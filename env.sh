# source this: ROS humble + devkit + venv deps (작업공간 위치 자동 인식)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; export WS
source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export PYTHONPATH="$WS/venv/lib/python3.10/site-packages:$PYTHONPATH"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
