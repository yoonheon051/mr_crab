#!/usr/bin/env python3
"""mr_crab 댄스: 팔 4축 춤 동작에 맞춰 그리퍼를 열고 닫고, 마지막에 홈 자세로 돌아온다."""
import json
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

MAX_SPEED = 1.2  # rad/s, 한 구간에서 가장 많이 움직이는 관절 기준
MIN_STEP = 0.7   # 구간 최소 시간(초)

# 그리퍼 간격(m). URDF 한계는 -0.011 ~ 0.02. 물건이 없으므로 닫을 때도 끝까지 조이지 않는다.
OPEN = 0.019
CLOSE = -0.006

home = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'home.json')))
names = list(home)
HOME = [home[n] for n in names]

# (joint1, joint2, joint3, joint4). joint2/3은 테이블에 닿지 않도록 위쪽 자세 위주로 제한.
READY = [0.0, -0.8, 0.8, 0.3]
SWAY_L = [0.6, -0.6, 0.6, 0.0]
SWAY_R = [-0.6, -0.6, 0.6, 0.0]
NOD_UP = [0.0, -0.6, 0.6, 1.2]
NOD_DN = [0.0, -0.6, 0.6, -0.8]
ROCK_A = [0.4, -0.2, 0.2, 0.8]
ROCK_B = [-0.4, -0.9, 1.0, 0.0]
SPIN_L = [1.5, -0.7, 0.7, 0.3]
SPIN_R = [-1.5, -0.7, 0.7, 0.3]

# (자세, 그리퍼). 그리퍼는 그 자세로 움직이기 시작할 때 보내는 명령이고 None이면 그대로 둔다.
STEPS = (
    [(READY, OPEN)]
    + [(SWAY_L, CLOSE), (SWAY_R, OPEN)] * 2 + [(READY, OPEN)]
    + [(NOD_UP, CLOSE), (NOD_DN, OPEN)] * 2 + [(READY, OPEN)]
    + [(ROCK_A, CLOSE), (ROCK_B, OPEN)] * 2 + [(READY, OPEN)]
    + [(SPIN_L, CLOSE), (SPIN_R, OPEN), (READY, CLOSE)]
    + [(HOME, None)]
)

rclpy.init()
node = Node('dance')
state = {}
node.create_subscription(JointState, '/joint_states', lambda m: state.update(zip(m.name, m.position)), 10)

t0 = time.time()
while not all(n in state for n in names) and time.time() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if not all(n in state for n in names):
    print('dance: /joint_states를 받지 못했습니다.', file=sys.stderr)
    sys.exit(1)

arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
gripper = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
if not arm.wait_for_server(timeout_sec=5.0):
    print('dance: arm_controller 액션 서버가 없습니다.', file=sys.stderr)
    sys.exit(1)
if not gripper.wait_for_server(timeout_sec=5.0):
    print('dance: gripper_controller 액션 서버가 없습니다.', file=sys.stderr)
    sys.exit(1)

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = names
prev = [state[n] for n in names]
t = 0.0
grip_events = []  # (시작 후 몇 초에, 그리퍼 간격)
for pos, grip in STEPS:
    if grip is not None:
        grip_events.append((t, grip))
    t += max(max(abs(a - b) for a, b in zip(pos, prev)) / MAX_SPEED, MIN_STEP)
    pt = JointTrajectoryPoint()
    pt.positions = list(pos)
    pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
    goal.trajectory.points.append(pt)
    prev = pos

print(f'dance: 시작 (총 {t:.1f}초, {len(STEPS)}개 동작, 그리퍼 {len(grip_events)}회)')
send = arm.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send, timeout_sec=5.0)
handle = send.result()
if handle is None or not handle.accepted:
    print('dance: 목표가 거부되었습니다.', file=sys.stderr)
    sys.exit(2)
start = time.time()
res = handle.get_result_async()

# 팔이 움직이는 동안 정해진 시각에 그리퍼 명령을 보낸다 (결과는 기다리지 않는다).
while not res.done() and time.time() - start < t + 5.0:
    while grip_events and time.time() - start >= grip_events[0][0]:
        g = GripperCommand.Goal()
        g.command.position = grip_events.pop(0)[1]
        g.command.max_effort = 0.0
        gripper.send_goal_async(g)
    rclpy.spin_once(node, timeout_sec=0.02)

if not res.done():
    print('dance: 시간 내에 완료되지 않았습니다.', file=sys.stderr)
    sys.exit(3)
code = res.result().result.error_code
print('dance: 완료 (홈 자세로 복귀)' if code == 0 else f'dance: 컨트롤러 error_code={code}')
