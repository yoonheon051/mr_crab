#!/bin/bash
# 다른 터미널에서 bringup.sh를 안전하게 종료(홈 이동 후 종료)시키고, 끝나면 알림을 띄운다.
STATUS=/tmp/mr_crab_home_status
PIDFILE=/tmp/mr_crab_bringup.pid
PID="$(cat "$PIDFILE" 2>/dev/null)"
if [ -z "$PID" ] || ! kill -0 "$PID" 2>/dev/null; then
  echo "실행 중인 bringup.sh가 없습니다."
  exit 1
fi
rm -f "$STATUS"
kill -TERM "$PID"
echo "종료 요청을 보냈습니다. (홈 이동 후 종료) 완료될 때까지 기다립니다..."

# bringup.sh가 완전히 끝날 때까지 최대 60초 대기
for _ in $(seq 120); do
  kill -0 "$PID" 2>/dev/null || break
  sleep 0.5
done

if kill -0 "$PID" 2>/dev/null; then
  MSG="⚠ 60초가 지나도 bringup이 종료되지 않았습니다. 전원을 끄지 마세요."
  RC=1
elif [ "$(cat "$STATUS" 2>/dev/null)" = "0" ]; then
  MSG="✅ 홈 자세로 이동하고 종료했습니다. 이제 전원을 꺼도 됩니다."
  RC=0
else
  MSG="⚠ 종료는 됐지만 홈 이동이 완료되지 않았을 수 있습니다. 팔 자세를 확인한 뒤 전원을 끄세요."
  RC=2
fi

printf '\a'                       # 터미널 벨
echo "$MSG"
command -v notify-send >/dev/null && notify-send "mr_crab" "$MSG" 2>/dev/null
exit $RC
