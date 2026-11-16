# OpenManipulator-X 전용 ROS 2 환경. source 해서 사용 (.bashrc는 수정하지 않음).
# .bashrc의 CycloneDDS/디스커버리 서버 설정은 이 PC에서 동작하지 않으므로 여기서만 덮어쓴다.
source /opt/ros/jazzy/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=0
export OMX_PORT=${OMX_PORT:-/dev/ttyACM0}
# 카메라 영상(프레임당 ~0.9MB)이 FastDDS 기본 공유메모리 세그먼트(~0.5MB)보다 커서 UDP로 밀리며 프레임이 유실되던 문제 해결 (fastdds_shm.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_shm.xml"
