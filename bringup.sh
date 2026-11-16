#!/bin/bash
# 하드웨어 bringup. 종료 신호(Ctrl+C, stop.sh, kill, 터미널 닫힘)를 받으면 홈 자세로 천천히 이동한 뒤 종료한다.
# 홈 이동 중 한 번 더 Ctrl+C를 누르면 이동을 중단하고 즉시 종료한다. (SIGKILL/전원 차단은 처리할 수 없다)
# 웹캠(C270) 노드(usb_cam)도 함께 띄운다: /camera1/image_raw. 끄려면 OMX_CAM=0 ./bringup.sh, 다른 카메라는 OMX_CAM=/dev/videoN ./bringup.sh
# 관절 각도 + 카메라 영상 + 터미널이 있는 웹 대시보드(dashboard.py)도 띄우고 브라우저 창을 연다. 끄려면 OMX_DASH=0 ./bringup.sh (포트: OMX_DASH_PORT, 기본 8765)
source "$(dirname "$0")/_run.sh"
DIR="$(cd "$(dirname "$0")" && pwd)"
PIDFILE=/tmp/mr_crab_bringup.pid
echo $$ > "$PIDFILE"   # stop.sh가 이 프로세스를 찾는 데 쓴다

# launch를 별도 세션(setsid)으로 띄워서 Ctrl+C가 launch에 바로 전달되지 않게 한다.
# 비대화형 셸의 백그라운드 자식은 SIGINT가 무시된 채 시작되므로 python으로 기본 동작을 복원한 뒤 exec 한다.
setsid python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp("ros2", ["ros2"] + sys.argv[1:])
' launch open_manipulator_bringup open_manipulator_x.launch.py port_name:=$OMX_PORT "$@" &
LAUNCH_PID=$!   # setsid 세션 리더이므로 PID == 프로세스 그룹 ID
HOME_PID=""
STOPPING=0

# 웹캠 노드 (usb_cam). 영상은 /camera1/image_raw 토픽으로 나간다. 실패해도 bringup에는 영향 없음
CAM_PID=""
CAM_DEV="${OMX_CAM:-/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0}"
if [ "$CAM_DEV" != "0" ]; then
  CAM_REAL="$(readlink -f "$CAM_DEV")"   # usb_cam은 심볼릭 링크를 못 풀어서 실제 경로로 넘긴다
  if [ -e "$CAM_REAL" ]; then
    setsid ros2 run usb_cam usb_cam_node_exe --ros-args \
      -p video_device:="$CAM_REAL" -p pixel_format:=yuyv2rgb \
      -p image_width:=640 -p image_height:=480 -p framerate:=30.0 \
      -p camera_name:=c270 -r image_raw:=/camera1/image_raw >/dev/null 2>&1 &
    CAM_PID=$!
    # 어두울 때 카메라가 스스로 fps를 낮추는 기능(exposure_dynamic_framerate)을 끈다. 실패해도 무시
    ( sleep 2; v4l2-ctl -d "$CAM_REAL" -c exposure_dynamic_framerate=0 >/dev/null 2>&1 ) &
  else
    echo "[bringup] 웹캠($CAM_DEV)을 찾지 못해 카메라 노드를 건너뜁니다."
  fi
fi

# 웹 대시보드: 로컬(127.0.0.1)에서만 열리고, 접속 URL에 일회용 토큰이 들어 있다. 실패해도 bringup에는 영향 없음
DASH_PID=""
if [ "${OMX_DASH:-1}" != "0" ]; then
  DASH_LOG=/tmp/mr_crab_dashboard.log
  setsid python3 "$DIR/dashboard.py" --port "${OMX_DASH_PORT:-8765}" >"$DASH_LOG" 2>&1 &
  DASH_PID=$!
  for _ in $(seq 40); do   # 서버가 URL을 출력할 때까지 최대 ~8초 대기
    DASH_URL="$(sed -n 's/^DASHBOARD_URL=//p' "$DASH_LOG" 2>/dev/null)"
    [ -n "$DASH_URL" ] && break
    kill -0 "$DASH_PID" 2>/dev/null || break
    sleep 0.2
  done
  if [ -n "$DASH_URL" ]; then
    echo "[bringup] 대시보드: $DASH_URL"
    if command -v google-chrome >/dev/null; then
      setsid google-chrome --app="$DASH_URL" --window-size=1200,900 >/dev/null 2>&1 &   # 주소창 없는 앱 창
    else
      setsid xdg-open "$DASH_URL" >/dev/null 2>&1 &
    fi
  else
    echo "[bringup] 대시보드를 시작하지 못했습니다. 로그: $DASH_LOG"
  fi
fi

shutdown() {
  if [ "$STOPPING" = 1 ]; then
    echo "[bringup] 홈 이동을 중단하고 즉시 종료합니다."
    [ -n "$HOME_PID" ] && kill "$HOME_PID" 2>/dev/null
    return
  fi
  STOPPING=1
  [ -n "$DASH_PID" ] && kill -USR1 "$DASH_PID" 2>/dev/null   # 홈 이동 중에는 대시보드 제어를 잠근다
  echo "[bringup] 종료 요청: 홈 자세로 이동합니다. (다시 Ctrl+C: 즉시 종료)"
  python3 "$DIR/go_home.py" &
  HOME_PID=$!
  wait "$HOME_PID"
  echo $? > /tmp/mr_crab_home_status   # stop.sh가 홈 이동 성공 여부를 알리는 데 쓴다
  echo "[bringup] launch를 종료합니다."
  kill -INT "$LAUNCH_PID" 2>/dev/null
}
trap shutdown INT TERM HUP

# launch가 끝날 때까지 대기 (trap에 의해 wait가 중단될 수 있으므로 반복)
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  wait "$LAUNCH_PID"
done

[ -n "$CAM_PID" ] && kill -INT -- -"$CAM_PID" 2>/dev/null
[ -n "$DASH_PID" ] && kill -INT -- -"$DASH_PID" 2>/dev/null

# 남은 자식 프로세스 정리: TERM 후 3초 지나도 남아 있으면 KILL
if pgrep -g "$LAUNCH_PID" >/dev/null; then
  kill -TERM -- -"$LAUNCH_PID" 2>/dev/null
  sleep 3
  pgrep -g "$LAUNCH_PID" >/dev/null && kill -KILL -- -"$LAUNCH_PID" 2>/dev/null
fi
rm -f "$PIDFILE"
exit 0
