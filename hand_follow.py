#!/usr/bin/env python3
"""손 따라가기: 카메라에 보이는 손바닥이 화면 중앙에 오도록 팔 관절을 움직이고, 주먹을 쥐면 그리퍼를 닫는다.

- /camera1/image_raw 를 MediaPipe HandLandmarker로 분석한다 (손바닥 중심 = 손목 + 네 손가락 뿌리 5점의 평균).
- 화면 좌우 오차 → joint1(좌우 회전), 상하 오차 → joint4(손목 기울기)를 비례제어한다. (카메라가 팔 끝에 달려 있다고 가정)
- 손이 화면에서 차지하는 크기(= 거리 대용)를 시작할 때의 크기로 유지하도록 팔을 앞뒤로 움직인다: 손이 작아지면(멀어지면) 다가가고,
  커지면(가까워지면) 물러난다. joint2·3을 평면 역기구학으로 풀어 카메라가 시선 방향으로 평행 이동하게 하고 joint4가 기울기를 보상한다.
- 제스처(손가락 접힘 기준, 서로 겹치지 않음):
    주먹(네 손가락 모두 접힘) → 그리퍼 닫기 / 손 펴기(접힌 손가락 1개 이하) → 그리퍼 열기
    중지·약지 접고 새끼는 폄(검지는 상관없음) → 팔이 그 자리에서 정지, 풀면 다시 추적 (그리퍼는 그대로)
    검지만 펴고 나머지 셋을 접음 → 브링업 자세(ready.json = bringup이 켜질 때 팔이 가는 자세 [0,-1,1,0])로 천천히 이동
      (이동 중 추적 중단, 도착 후 손가락을 풀 때까지 대기). 접힌 홈 자세로 보내려면 --pose-file home.json
- 주먹(네 손가락 모두 접힘) → 그리퍼 닫기, 손 펴기(접힌 손가락 1개 이하) → 열기. 몇 프레임 연속일 때만 바꾼다.
- 손이 사라지면 그 자리에서 멈춘다(홈으로 가지 않는다). 종료(Ctrl+C)해도 그 자세 그대로다.
- 인식 결과를 그린 영상은 /camera1/image_hand 로 나가고, 대시보드가 있으면 자동으로 그 영상을 보여 준다.
사용: ./hand_follow.sh [--dry-run] [--pan-sign -1] [--tilt-sign -1] ...   (--help 참고)
"""
import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "hand_libs"))   # pip --target 으로 설치한 mediapipe

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from mediapipe.tasks.python import BaseOptions, vision
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4"]
GRIP_OPEN, GRIP_CLOSE = 0.019, -0.006      # dance.py와 같은 값 (m)
HFOV, VFOV = 1.05, 0.80                    # C270 대략적인 화각(rad). 화면 오차 → 각도 오차 환산용
# 손 랜드마크 번호: 손목 0, 손가락 (끝, PIP): 검지 8/6, 중지 12/10, 약지 16/14, 새끼 20/18, 뿌리(MCP) 5/9/13/17
FINGERS = [(8, 6), (12, 10), (16, 14), (20, 18)]
PALM_PTS = [0, 5, 9, 13, 17]
SKELETON = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
            (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


class OneEuro:
    """One Euro 필터: 가만히 있을 땐 강하게 평활(떨림 제거), 빠르게 움직일 땐 약하게(지연 최소화)."""

    def __init__(self, min_cutoff, beta, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = self.dx = self.t = None

    @staticmethod
    def _alpha(dt, fc):
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * fc) / dt)

    def reset(self):
        self.x = self.dx = self.t = None

    def __call__(self, x, t):
        if self.x is None:
            self.x, self.dx, self.t = x, 0.0, t
            return x
        dt = t - self.t
        if dt <= 1e-6:
            return self.x
        self.t = t
        dx = (x - self.x) / dt
        self.dx += self._alpha(dt, self.d_cutoff) * (dx - self.dx)
        fc = self.min_cutoff + self.beta * abs(self.dx)
        self.x += self._alpha(dt, fc) * (x - self.x)
        return self.x


def slew(cur, want, max_delta):
    """cur를 want 쪽으로 최대 max_delta만큼만 이동 (가속도 제한용)."""
    return cur + max(-max_delta, min(max_delta, want - cur))


# ── 평면 기구학 (joint2/3/4는 모두 y축 회전, URDF open_manipulator_x_arm.urdf.xacro). (x, z) = (앞, 위), 단위 m ──
X2, Z2 = 0.012, 0.0765            # joint2 위치 (베이스 기준, z는 ROBOTIS 사양의 어깨 높이)
LA, LB, LC = (0.024, 0.128), (0.124, 0.0), (0.126, 0.0)   # joint2→3, joint3→4, joint4→끝점(end effector)
JLIM = {"joint2": (-1.5, 1.5), "joint3": (-1.5, 1.4), "joint4": (-1.7, 1.97)}
LEAD2 = 0.35                      # 앞뒤 이동 중 목표가 실제 자세보다 앞서갈 수 있는 최대 관절 각도 차(rad)


def _rot(th, v):                  # y축 회전: 양(+)이면 위쪽(+z) 벡터가 앞쪽(+x)으로 기운다
    c, s_ = math.cos(th), math.sin(th)
    return (v[0] * c + v[1] * s_, -v[0] * s_ + v[1] * c)


def fk(q2, q3, q4):
    """(joint4 위치, 끝점 위치) 각각 (x, z). 카메라 자세(피치) φ = q2+q3+q4, 시선 방향은 (cosφ, -sinφ)."""
    a, b, c = _rot(q2, LA), _rot(q2 + q3, LB), _rot(q2 + q3 + q4, LC)
    p4 = (X2 + a[0] + b[0], Z2 + a[1] + b[1])
    return p4, (p4[0] + c[0], p4[1] + c[1])


def depth_dq(q2, q3, q4, dist):
    """카메라(link5)를 자세는 그대로 두고 시선 방향으로 dist(m) 평행 이동시키는 관절 변화 (dq2, dq3, dq4)."""
    phi = q2 + q3 + q4
    want = (dist * math.cos(phi), -dist * math.sin(phi))
    e = 1e-6
    p0 = fk(q2, q3, q4)[0]
    c2 = [(fk(q2 + e, q3, q4)[0][i] - p0[i]) / e for i in (0, 1)]   # d p4 / d q2
    c3 = [(fk(q2, q3 + e, q4)[0][i] - p0[i]) / e for i in (0, 1)]   # d p4 / d q3
    J = np.array([[c2[0], c3[0]], [c2[1], c3[1]]])
    lam = 0.01                                                       # 특이점 근처 보호(감쇠 최소제곱)
    dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(2), np.array(want))
    return float(dq[0]), float(dq[1]), float(-(dq[0] + dq[1]))      # joint4가 피치를 보상


def analyze(lms, aspect=1.0):
    """랜드마크 21개 → (손바닥 중심 (x, y) 0~1, 접힌 손가락 수, 손 크기, 손가락별 접힘 [검지, 중지, 약지, 새끼]).
    aspect=가로/세로 (정규화 좌표는 가로세로 축척이 달라 거리 비교 전에 보정). 손 크기 = 손바닥 대각선 길이/화면 높이:
    (손목↔중지 뿌리)와 (새끼↔검지 뿌리)를 합쳐서 손을 돌려도 덜 변한다. 거리가 멀수록 작다."""
    p = np.array([[l.x, l.y] for l in lms])
    center = p[PALM_PTS].mean(axis=0)
    p = p * [aspect, 1.0]
    size = float(math.hypot(np.linalg.norm(p[9] - p[0]), np.linalg.norm(p[5] - p[17])))
    wrist = p[0]
    flags = [bool(np.linalg.norm(p[t] - wrist) < np.linalg.norm(p[b] - wrist)) for t, b in FINGERS]   # 검지, 중지, 약지, 새끼 접힘 여부
    return (float(center[0]), float(center[1])), int(sum(flags)), size, flags


def classify(flags):
    """손가락별 접힘 [검지, 중지, 약지, 새끼] → 'fist' | 'home' | 'hold' | None. 서로 겹치지 않게 나눈다."""
    idx, mid, ring, pinky = flags
    if all(flags):
        return "fist"
    if not idx and mid and ring and pinky:
        return "home"            # 검지만 펴고 나머지는 접음
    if mid and ring and not pinky:
        return "hold"            # 중지·약지만 접음 (검지는 폄/접음 모두 허용)
    return None


HOME_SPEED = 0.35   # rad/s, go_home.py와 같은 속도로 천천히


def make_landmarker(model, running_mode):
    return vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model)), running_mode=running_mode, num_hands=1,
        min_hand_detection_confidence=0.6, min_hand_presence_confidence=0.5, min_tracking_confidence=0.5))


class HandFollow(Node):
    def __init__(self, a):
        super().__init__("hand_follow")
        self.a = a
        self.lock = threading.Lock()
        self.frame = None            # (seq, rgb ndarray)
        self.seq = 0
        self.cur = {}                # /joint_states
        self.target = None           # 현재 명령 중인 관절 목표. 손을 (다시) 볼 때마다 현재 위치로 다시 맞춘다
        self._grip_warn = 0.0
        self.size = None             # 손 크기(지수평활)
        self.ref = a.ref_size        # 유지할 손 크기. None이면 시작할 때 손 크기를 기준으로 삼는다
        self._ref_buf = []
        self.depth_pos = 0.0         # 이번 추적 구간에서 앞(+)/뒤(-)로 움직인 누적 거리(m)
        self.depth_note = ""
        self.fx = OneEuro(a.smooth, a.smooth_beta)            # 손 위치(정규화 좌표) 필터
        self.fy = OneEuro(a.smooth, a.smooth_beta)
        self.fs = OneEuro(a.smooth * 0.35, a.smooth_beta * 0.4)  # 손 크기(로그) 필터: 손목/손 기울임에 크기가 쉽게 ±20% 변하므로 훨씬 강하게
        self.vel = {"pan": 0.0, "tilt": 0.0, "depth": 0.0}    # 현재 속도 명령 (가속도 제한으로 부드럽게 변화)
        self.sent = None             # 마지막으로 컨트롤러에 보낸 목표
        self.last_pub = 0.0
        self.n_pub = 0
        self.log = open(a.log, "w") if a.log else None
        if self.log:
            self.log.write("t,raw_x,raw_y,raw_size,x,y,size,vel_pan,vel_tilt,vel_depth,t_j1,t_j2,t_j3,t_j4,c_j1,c_j2,c_j3,c_j4,published\n")
        self._ts = 0
        self.grip_closed = False
        self.fist_n = self.open_n = 0
        self.hold_n = self.rel_n = 0
        self.holding = False         # 중지·약지 접기 제스처로 팔을 멈춘 상태
        try:
            self.home = json.load(open(a.pose_file))
            self.pose_name = Path(a.pose_file).stem
        except Exception as e:
            self.home = None
            self.pose_name = "?"
            print(f"[hand] {a.pose_file}을 읽지 못해 검지 제스처(자세 이동)는 비활성입니다: {e}", file=sys.stderr, flush=True)
        self.homing = False          # 홈으로 이동 중 (추적/목표 갱신 중단)
        self.homing_end = 0.0
        self.after_home = False      # 홈 도착 후 검지 제스처를 풀 때까지 추적하지 않고 대기
        self.point_n = self.point_rel = 0
        self.point_armed = True      # 홈 제스처를 한 번 쓴 뒤 풀었다가 다시 해야 재발동
        self.last_seen = 0.0
        self.last_t = None
        self.create_subscription(Image, a.topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.img_pub = self.create_publisher(Image, "/camera1/image_hand", qos_profile_sensor_data)
        self.grip = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.lm = make_landmarker(a.model, vision.RunningMode.VIDEO)

    def on_joints(self, m):
        self.cur.update(zip(m.name, m.position))

    def on_image(self, m):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        try:   # 예외가 spin 스레드를 죽이면 영상이 조용히 멈추므로 여기서 막는다
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
            rgb = a if m.encoding == "rgb8" else a[:, :, ::-1]
            with self.lock:
                self.seq += 1
                self.frame = (self.seq, np.ascontiguousarray(rgb))
        except Exception as e:
            self.get_logger().warn(f"이미지 처리 실패: {e}", throttle_duration_sec=5.0)

    # ── 제어 ──
    def drive(self, center, dt):
        a = self.a
        ex, ey = center[0] - 0.5, center[1] - 0.5
        ex = 0.0 if abs(ex) < a.deadband else ex - math.copysign(a.deadband, ex)   # 데드밴드 밖 초과분만 사용
        ey = 0.0 if abs(ey) < a.deadband else ey - math.copysign(a.deadband, ey)
        rate = lambda e, fov, sign: max(-a.max_rate, min(a.max_rate, sign * a.gain * e * fov))
        lead = a.max_rate * 0.6   # 목표가 실제 위치보다 앞서갈 수 있는 최대 거리(rad)
        for key, j, e, fov, sign, lo, hi in (("pan", a.pan_joint, ex, HFOV, a.pan_sign, -a.pan_range, a.pan_range),
                                             ("tilt", a.tilt_joint, ey, VFOV, a.tilt_sign, a.tilt_min, a.tilt_max)):
            c = self.cur[j]
            self.vel[key] = slew(self.vel[key], rate(e, fov, sign), a.accel * dt)   # 가속도 제한: 속도가 갑자기 바뀌지 않게
            t = self.target[j] + self.vel[key] * dt
            t = min(max(t, c - lead), c + lead)        # 안티 와인드업: 팔이 못 따라가도 목표가 계속 달아나지 않게
            t = min(max(t, min(lo, c)), max(hi, c))    # 이동 범위. 이미 범위 밖이면 튀지 않고 안쪽으로만 돌아오게
            self.target[j] = t

    def depth_step(self, size, dt):
        """손 크기를 ref로 유지하도록 카메라를 시선 방향으로 앞뒤 이동. 위험하면 그 걸음은 건너뛴다."""
        a = self.a
        self.size = size   # 이미 One Euro로 평활된 값
        if self.ref is None:       # 시작할 때 손 크기 15프레임 평균을 기준으로
            self._ref_buf.append(self.size)
            if len(self._ref_buf) >= 15:
                self.ref = float(np.mean(self._ref_buf))
                print(f"[hand] 기준 손 크기 {self.ref:.3f} 로 고정. 이보다 작아지면 다가가고 커지면 물러납니다.", flush=True)
            self.depth_note = "calibrating"
            return
        e = (self.ref - self.size) / self.ref          # + : 기준보다 작음(멀다) → 다가가기
        e = 0.0 if abs(e) < a.depth_deadband else e - math.copysign(a.depth_deadband, e)
        v = max(-a.max_depth_speed, min(a.max_depth_speed, a.depth_sign * a.depth_gain * e))
        v = self.vel["depth"] = slew(self.vel["depth"], v, a.depth_accel * dt)
        if abs(v) < 1e-4:
            self.vel["depth"] = 0.0
            self.depth_note = "hold"
            return
        t = self.target
        dist = v * dt
        d2, d3, d4 = depth_dq(t["joint2"], t["joint3"], t["joint4"], dist)
        cand = {"joint2": t["joint2"] + d2, "joint3": t["joint3"] + d3, "joint4": t["joint4"] + d4}
        why = None
        for j, c in cand.items():
            lo, hi = JLIM[j]
            if j == "joint4":
                lo, hi = max(lo, a.tilt_min), min(hi, a.tilt_max)
            if not (min(lo, t[j]) <= c <= max(hi, t[j])):
                why = f"{j} 한계"
            elif abs(c - self.cur[j]) > LEAD2 and abs(c - self.cur[j]) > abs(t[j] - self.cur[j]):
                why = "팔이 아직 못 따라옴"
        p4, tip = fk(cand["joint2"], cand["joint3"], cand["joint4"])
        p4_0, tip_0 = fk(t["joint2"], t["joint3"], t["joint4"])
        if min(p4[1], tip[1]) < a.min_height and min(p4[1], tip[1]) < min(p4_0[1], tip_0[1]):
            why = "최소 높이"
        if abs(self.depth_pos + dist) > a.max_travel and abs(self.depth_pos + dist) > abs(self.depth_pos):
            why = "최대 이동 거리"
        if why:
            self.depth_note = f"limit: {why}"
            return
        t.update(cand)
        self.depth_pos += dist
        self.depth_note = f"{'fwd' if v > 0 else 'back'} {v * 100:+.1f}cm/s"

    def maybe_publish(self, now):
        """목표가 실제로 바뀌었을 때만 보낸다. 같은 목표를 계속 다시 보내면 컨트롤러가 매번 측정값(잡음 포함)에서
        궤적을 다시 계산해 제자리에서 떨 수 있다. 안 보내면 컨트롤러는 마지막 목표 자세를 유지한다."""
        if now - self.last_pub < 1.0 / self.a.pub_rate:
            return False
        if self.sent is not None and max(abs(self.target[j] - self.sent[j]) for j in ARM) < self.a.pub_eps:
            return False
        self.last_pub = now
        self.sent = dict(self.target)
        self.n_pub += 1
        if not self.a.dry_run:
            msg = JointTrajectory()
            msg.joint_names = ARM
            pt = JointTrajectoryPoint()
            pt.positions = [self.target[j] for j in ARM]
            pt.time_from_start = Duration(sec=int(self.a.horizon), nanosec=int((self.a.horizon % 1) * 1e9))
            msg.points = [pt]
            self.traj_pub.publish(msg)
        return True

    def set_gripper(self, closed):
        if not (self.a.dry_run or self.a.no_gripper):
            if not self.grip.wait_for_server(timeout_sec=0.0):   # 못 보냈으면 상태를 바꾸지 않아 다음 프레임에 재시도
                if time.monotonic() - self._grip_warn > 3.0:
                    print("[hand] gripper_controller 액션 서버가 없습니다. 계속 재시도합니다.", file=sys.stderr, flush=True)
                    self._grip_warn = time.monotonic()
                return
            g = GripperCommand.Goal()
            g.command.position = GRIP_CLOSE if closed else GRIP_OPEN
            g.command.max_effort = 0.0
            self.grip.send_goal_async(g)
        self.grip_closed = closed
        print(f"[hand] 그리퍼 {'닫기 (주먹)' if closed else '열기'}", flush=True)

    def pose_label(self):
        return {"ready": "브링업", "home": "홈"}.get(self.pose_name, self.pose_name)

    def start_home(self, now):
        a = self.a
        if self.home is None or not all(j in self.cur for j in ARM):
            return
        delta = max(abs(self.home[j] - self.cur[j]) for j in ARM)
        if delta < 0.02:
            print(f"[hand] 이미 {self.pose_label()} 자세입니다.", flush=True)
            self.after_home = True
            return
        T = min(max(delta / a.home_speed, 3.0), 20.0)
        if a.dry_run:
            print(f"[hand] 검지 제스처 인식 (dry-run이라 이동하지 않음, 실제로는 {self.pose_label()} 자세로 {T:.1f}초 걸림)", flush=True)
            self.after_home = True
            return
        msg = JointTrajectory()
        msg.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [float(self.home[j]) for j in ARM]
        pt.time_from_start = Duration(sec=int(T), nanosec=int((T % 1) * 1e9))
        msg.points = [pt]
        self.traj_pub.publish(msg)
        self.target = {j: float(self.home[j]) for j in ARM}
        self.sent = dict(self.target)
        self.vel = {"pan": 0.0, "tilt": 0.0, "depth": 0.0}
        self.homing, self.homing_end = True, now + T + 2.0
        print(f"[hand] 검지 제스처 — {self.pose_label()} 자세로 이동합니다 ({T:.1f}초, 최대 변위 {delta:.2f}rad). 중지·약지를 접으면 중단.", flush=True)

    def update_homing(self, now):
        """홈 이동 도착/시간 초과 확인. 손이 안 보여도 매 프레임 호출한다."""
        if not self.homing:
            return
        arrived = all(abs(self.cur[j] - self.home[j]) < 0.03 for j in ARM)
        if arrived or now > self.homing_end:
            self.homing, self.after_home = False, True
            self.target = {j: self.cur[j] for j in ARM}   # 도착한 자세에서 다시 시작
            self.sent = dict(self.target)
            self.depth_pos = 0.0
            self.size = None
            print(f"[hand] {self.pose_label()} 자세 도착. 검지를 풀면 다시 따라갑니다." if arrived else
                  "[hand] 이동 시간 초과 — 현재 자세에서 멈춥니다.", flush=True)

    def control(self, raw_center, raw_size, now, dt, absent, hold=False):
        """손 위치/크기 → 필터 → 관절 목표 갱신 → (필요하면) 명령 전송. 반환: (필터된 중심, 필터된 크기, 성공 여부)."""
        if absent:   # 다시 보인 손은 필터를 처음부터
            for f in (self.fx, self.fy, self.fs):
                f.reset()
            self.vel = {"pan": 0.0, "tilt": 0.0, "depth": 0.0}
        if self.a.smooth > 0:
            center = (self.fx(raw_center[0], now), self.fy(raw_center[1], now))
            size = math.exp(self.fs(math.log(raw_size), now))
        else:
            center, size = raw_center, raw_size
        if self.homing:   # 홈 이동 중에는 목표를 건드리거나 명령을 보내지 않는다 (진행 중인 이동을 덮어쓰지 않게)
            return center, size, True
        if self.target is None or absent:   # 처음이거나 손을 놓쳤다 다시 봤을 때: 지금 자세에서 다시 시작
            if not all(j in self.cur for j in ARM):   # (그 사이 대시보드 등으로 팔이 움직였어도 옛 자세로 튀지 않게)
                return center, size, False
            self.target = {j: self.cur[j] for j in ARM}
            self.depth_pos = 0.0
            self.size = None
        if hold:   # 정지: 필터는 계속 갱신(풀 때 튀지 않게)하지만 목표는 움직이지 않는다
            self.vel = {"pan": 0.0, "tilt": 0.0, "depth": 0.0}
        else:
            self.drive(center, dt)
            if not self.a.no_depth:
                self.depth_step(size, dt)
        published = self.maybe_publish(now)
        if self.log:
            self.log.write(",".join(f"{v:.5f}" for v in (now, *raw_center, raw_size, *center, size, self.vel["pan"],
                           self.vel["tilt"], self.vel["depth"], *[self.target[j] for j in ARM],
                           *[self.cur[j] for j in ARM])) + f",{int(published)}\n")
        return center, size, True

    def step(self, rgb):
        now = time.monotonic()
        self._ts = max(int(now * 1000), self._ts + 1)   # VIDEO 모드는 타임스탬프가 반드시 증가해야 한다
        res = self.lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), self._ts)
        out = np.ascontiguousarray(rgb[:, :, ::-1])   # 그리기용 BGR
        h, w = out.shape[:2]
        cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 0), cv2.MARKER_CROSS, 24, 1)
        label = "no hand"
        self.update_homing(now)
        if res.hand_landmarks:
            lms = res.hand_landmarks[0]
            raw_center, curled, raw_size, flags = analyze(lms, w / h)
            absent = now - self.last_seen > 0.5
            self.last_seen = now
            dt = min(now - self.last_t, 0.2) if self.last_t else 0.0
            if absent:
                self.hold_n = self.rel_n = 0
                self.holding = False
            g = classify(flags)
            want_point = g == "home"
            self.point_n = self.point_n + 1 if want_point else 0
            self.point_rel = self.point_rel + 1 if not want_point else 0
            if absent:
                self.point_n = self.point_rel = 0
            if self.point_rel >= self.a.confirm:    # 검지 제스처를 풀었으면: 재발동 가능, 홈 대기 해제
                self.point_armed = True
                self.after_home = False
            if want_point and self.point_n >= self.a.home_confirm and self.point_armed and not self.homing:
                self.point_armed = False
                self.start_home(now)
            # 정지 제스처 (연속 프레임으로 확정)
            want_hold = g == "hold"
            self.hold_n = self.hold_n + 1 if want_hold else 0
            self.rel_n = self.rel_n + 1 if not want_hold else 0
            if self.hold_n >= self.a.confirm and not self.holding:
                self.holding = True
                if self.homing:   # 홈 이동 중이면 중단하고 그 자리에서 멈춤
                    self.homing = False
                    print("[hand] 자세 이동을 중단합니다.", flush=True)
                if self.target is not None and all(j in self.cur for j in ARM):
                    self.target = {j: self.cur[j] for j in ARM}   # 지금 자세에서 바로 멈추게(목표가 앞서 있던 만큼 되돌아가지 않도록 현재 위치로)
                    self.sent = None                               # 그 목표를 즉시 컨트롤러에 전달
                print("[hand] 정지 (중지·약지 접음) — 팔을 이 자세로 유지합니다.", flush=True)
            elif self.rel_n >= self.a.confirm and self.holding:
                self.holding = False
                print("[hand] 정지 해제 — 다시 따라갑니다.", flush=True)
            center, size, ok = self.control(raw_center, raw_size, now, dt, absent, hold=self.holding or self.after_home)
            if not ok:
                return out, "waiting /joint_states"
            # 주먹/펴기 판정은 연속 프레임으로 (한두 프레임 오인식 방지)
            self.fist_n = self.fist_n + 1 if curled >= 4 else 0
            self.open_n = self.open_n + 1 if curled <= 1 else 0
            if self.fist_n >= self.a.confirm and not self.grip_closed:
                self.set_gripper(True)
            elif self.open_n >= self.a.confirm and self.grip_closed:
                self.set_gripper(False)
            pts = [(int(l.x * w), int(l.y * h)) for l in lms]
            for i, j in SKELETON:
                cv2.line(out, pts[i], pts[j], (0, 200, 0), 2)
            for q in pts:
                cv2.circle(out, q, 3, (0, 0, 255), -1)
            cp = (int(center[0] * w), int(center[1] * h))
            cv2.circle(out, cp, 9, (0, 255, 255), 2)
            cv2.line(out, (w // 2, h // 2), cp, (0, 255, 255), 1)
            label = f"{'HOMING ' if self.homing else 'HOME-WAIT ' if self.after_home else ''}{'HOLD ' if self.holding else ''}{'FIST' if curled >= 4 else 'OPEN' if curled <= 1 else '...'} curled={curled}  grip={'CLOSED' if self.grip_closed else 'open'}  {'' if self.a.no_depth else f'size={size:.2f}/{(self.ref or 0):.2f} '}{self.depth_note}"
        else:
            self.fist_n = self.open_n = 0
            self.hold_n = self.rel_n = 0
            self.point_n = 0
        self.last_t = now
        cv2.putText(out, label + ("  [DRY RUN]" if self.a.dry_run else ""), (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)
        return out, label

    def publish_image(self, bgr):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.height, m.width = bgr.shape[:2]
        m.encoding, m.step = "bgr8", m.width * 3
        m.data = bgr.tobytes()
        self.img_pub.publish(m)


def run(a, node_cls, banner):
    """공통 실행 루프: 카메라 프레임마다 node.step()을 돌리고 인식 영상을 발행한다."""
    if not a.model.exists():
        sys.exit(f"모델 파일이 없습니다: {a.model}")
    rclpy.init()
    node = node_cls(a)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print(f"[hand] 시작 ({'DRY RUN, 로봇은 움직이지 않음' if a.dry_run else '로봇 제어 중'}). {banner}. 종료: Ctrl+C", flush=True)
    last_seq, last_log, lost_said, frames, last_pub_n = 0, time.monotonic(), False, 0, 0
    try:
        while rclpy.ok():
            with node.lock:
                fr = node.frame
            if fr is None or fr[0] == last_seq:
                time.sleep(0.003)
                continue
            last_seq = fr[0]
            out, label = node.step(fr[1])
            node.publish_image(out)
            now = time.monotonic()
            frames += 1
            if now - last_log >= 5.0:   # 5초마다 상태 한 줄
                print(f"[hand] {frames / (now - last_log):.1f} fps | 명령 {node.n_pub - last_pub_n}회/5s | {label}", flush=True)
                last_pub_n = node.n_pub
                frames, last_log = 0, now
            if label == "no hand" and node.target is not None and not lost_said and now - node.last_seen > 0.5:
                print("[hand] 손을 놓쳤습니다 — 현재 자세에서 멈춥니다.", flush=True)
                lost_said = True
            elif label != "no hand":
                lost_said = False
    except KeyboardInterrupt:
        pass
    print("[hand] 종료 (팔은 현재 자세 그대로)", flush=True)
    node.lm.close()
    if node.log:
        node.log.close()
    rclpy.try_shutdown()




def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/camera1/image_raw")
    ap.add_argument("--model", type=Path, default=HERE / "models" / "hand_landmarker.task")
    ap.add_argument("--dry-run", action="store_true", help="로봇에는 명령을 보내지 않고 인식/계산 결과만 보여 준다")
    ap.add_argument("--no-gripper", action="store_true", help="그리퍼는 움직이지 않는다")
    ap.add_argument("--pan-joint", default="joint1")
    ap.add_argument("--tilt-joint", default="joint4")
    ap.add_argument("--pan-sign", type=float, default=-1.0, help="손이 화면 오른쪽일 때 pan 관절 방향. joint1은 위에서 볼 때 반시계가 +이므로 오른쪽으로 돌려면 -1 (카메라가 팔 앞쪽을 볼 때). 반대로 가면 +1")
    ap.add_argument("--tilt-sign", type=float, default=1.0, help="손이 화면 아래쪽일 때 tilt 관절 방향. 반대로 가면 -1")
    ap.add_argument("--gain", type=float, default=0.8, help="비례 이득 (1/s). 클수록 빠르게 따라가지만 흔들릴 수 있다")
    ap.add_argument("--max-rate", type=float, default=0.4, help="관절 최대 속도 (rad/s)")
    ap.add_argument("--deadband", type=float, default=0.06, help="화면 중앙 허용 오차 (화면 크기 비율)")
    ap.add_argument("--pan-range", type=float, default=1.5, help="pan 관절 이동 범위 ±rad")
    ap.add_argument("--tilt-min", type=float, default=-1.0)
    ap.add_argument("--tilt-max", type=float, default=1.5)
    ap.add_argument("--no-depth", action="store_true", help="앞뒤 이동(손 크기 추적)을 끈다")
    ap.add_argument("--depth-sign", type=float, default=1.0, help="손이 작아질 때 다가가는 방향. 반대로 움직이면 -1")
    ap.add_argument("--depth-gain", type=float, default=0.3, help="앞뒤 이동 이득 (m/s per 100%% 크기 오차)")
    ap.add_argument("--max-depth-speed", type=float, default=0.06, help="앞뒤 최대 속도 (m/s)")
    ap.add_argument("--depth-deadband", type=float, default=0.18, help="손 크기 허용 오차 (기준 대비 비율)")
    ap.add_argument("--ref-size", type=float, default=None, help="유지할 손 크기. 생략하면 시작할 때 크기를 기준으로 삼는다")
    ap.add_argument("--min-height", type=float, default=0.05, help="joint4/끝점이 내려갈 수 있는 최소 높이(m, 베이스 기준)")
    ap.add_argument("--max-travel", type=float, default=0.12, help="한 번의 추적 동안 앞뒤로 움직일 수 있는 최대 거리(m)")
    ap.add_argument("--smooth", type=float, default=1.0, help="손 위치/크기 필터의 최소 차단주파수(Hz). 작을수록 부드럽지만 반응이 느려짐. 0이면 필터 끔")
    ap.add_argument("--smooth-beta", type=float, default=4.0, help="빠르게 움직일 때 필터를 풀어 주는 정도 (클수록 지연 감소)")
    ap.add_argument("--accel", type=float, default=1.5, help="관절 속도 명령의 최대 가속도 (rad/s^2)")
    ap.add_argument("--depth-accel", type=float, default=0.3, help="앞뒤 속도 명령의 최대 가속도 (m/s^2)")
    ap.add_argument("--pub-rate", type=float, default=30.0, help="컨트롤러로 보내는 최대 빈도 (Hz). 실측: 10Hz는 궤적 재시작이 잦아 움직임이 톱니 모양이 되고, 30Hz가 가장 매끄러움")
    ap.add_argument("--pub-eps", type=float, default=0.001, help="목표가 이만큼(rad) 이상 바뀔 때만 보낸다. 0.0015rad가 모터 1틱")
    ap.add_argument("--horizon", type=float, default=0.05, help="각 명령을 도달시킬 시간(s). 실측 지연: 0.4s→650ms, 0.1s→280ms, 0.05s→135ms. 너무 길면 팔이 한참 늦게 따라옴")
    ap.add_argument("--log", default=None, help="분석용 CSV 기록 경로 (예: /tmp/hand.csv)")
    ap.add_argument("--pose-file", type=Path, default=HERE / "ready.json", help="검지 제스처로 이동할 자세(JSON, joint1~4 rad). 기본 ready.json=bringup 시작 자세, 접힌 홈은 home.json")
    ap.add_argument("--home-confirm", type=int, default=10, help="검지 제스처를 자세 이동으로 인정하는 연속 프레임 수 (큰 동작이라 --confirm보다 길게)")
    ap.add_argument("--home-speed", type=float, default=HOME_SPEED, help="자세 이동 속도 (rad/s)")
    ap.add_argument("--confirm", type=int, default=4, help="주먹/펴기를 인정하는 연속 프레임 수")
    a = ap.parse_args()
    run(a, HandFollow, f"pan={a.pan_joint}*{a.pan_sign:+.0f} tilt={a.tilt_joint}*{a.tilt_sign:+.0f}")


if __name__ == "__main__":
    main()
