#!/bin/bash
# 그리퍼 모터(ID 15) 재부팅: 손으로 돌려서 멀티턴 카운터가 어긋났을 때 사용. bringup은 꺼 둘 것.
source "$(dirname "$0")/_run.sh"
python3 - <<PY
import time
from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler("$OMX_PORT")
if not p.openPort() or not p.setBaudRate(1000000):
    raise SystemExit("$OMX_PORT 를 열지 못했습니다. bringup 이 켜져 있으면 먼저 끄세요.")
h = PacketHandler(2.0); h.reboot(p, 15); time.sleep(1.5)
v = h.read4ByteTxRx(p, 15, 132)[0]; v = v - 2**32 if v >= 2**31 else v
print("gripper raw position:", v, "(정상 범위 약 1420~3180)")
PY
