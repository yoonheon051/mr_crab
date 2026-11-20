#!/usr/bin/env python3
"""팔 4축을 home.json 자세로 천천히 이동시키고, 도착하면 0으로 종료한다. 그리퍼는 건드리지 않는다."""
import json
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

MAX_SPEED = 0.35  # rad/s, 가장 많이 움직이는 관절 기준
MIN_TIME = 3.0
MAX_TIME = 20.0

home = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'home.json')))
names = list(home)

rclpy.init()
node = Node('go_home')
state = {}
node.create_subscription(
    JointState, '/joint_states',
    lambda m: state.update(zip(m.name, m.position)), 10)

t0 = time.time()
while not all(n in state for n in names) and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if not all(n in state for n in names):
    print('go_home: /joint_states를 받지 못해 홈 이동을 건너뜁니다.', file=sys.stderr)
    sys.exit(1)

client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
if not client.wait_for_server(timeout_sec=5.0):
    print('go_home: arm_controller 액션 서버가 없어 홈 이동을 건너뜁니다.', file=sys.stderr)
    sys.exit(1)

delta = max(abs(home[n] - state[n]) for n in names)
T = min(max(delta / MAX_SPEED, MIN_TIME), MAX_TIME)
if delta < 0.02:
    print('go_home: 이미 홈 자세입니다.')
    sys.exit(0)

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = names
pt = JointTrajectoryPoint()
pt.positions = [home[n] for n in names]
pt.time_from_start = Duration(sec=int(T), nanosec=int((T % 1) * 1e9))
goal.trajectory.points = [pt]

print(f'go_home: 홈으로 이동 ({T:.1f}초, 최대 변위 {delta:.2f}rad)')
send = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
handle = send.result()
if handle is None or not handle.accepted:
    print('go_home: 목표가 거부되었습니다.', file=sys.stderr)
    sys.exit(2)
res = handle.get_result_async()
rclpy.spin_until_future_complete(node, res, timeout_sec=T + 5.0)
if res.result() is None:
    print('go_home: 시간 내에 완료되지 않았습니다.', file=sys.stderr)
    sys.exit(3)
code = res.result().result.error_code
if code != 0:   # 0 이 아니면 stop.sh 가 "전원을 꺼도 됩니다" 대신 자세 확인 경고를 띄우도록 실패 코드로 끝낸다
    print(f'go_home: 컨트롤러 error_code={code} (홈에 도착하지 못했을 수 있음)', file=sys.stderr)
    sys.exit(4)
print('go_home: 완료')
sys.exit(0)
