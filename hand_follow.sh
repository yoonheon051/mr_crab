#!/bin/bash
# 손 따라가기 실행 (bringup이 켜진 상태에서). 옵션은 ./hand_follow.sh --help 참고. 처음엔 --dry-run으로 인식만 확인하세요.
source "$(dirname "$0")/env.sh"
exec python3 "$(dirname "$0")/hand_follow.py" "$@"
