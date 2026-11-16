#!/bin/bash
# MoveIt + RViz. bringup.sh를 먼저 실행해 둔 상태에서 사용. 예: ./moveit.sh [start_rviz:=false]
source "$(dirname "$0")/_run.sh"
exec ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py "$@"
