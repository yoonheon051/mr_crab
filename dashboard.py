#!/usr/bin/env python3
"""mr_crab 대시보드: 관절 각도(조작 가능) + 카메라 영상 + 터미널을 한 웹페이지로 보여준다.

- /joint_states, /camera1/image_raw 를 구독해서 웹소켓(/ws)으로 브라우저에 보낸다.
- 페이지의 슬라이더/버튼은 /ws로 move·gripper·home·stop 명령을 보내고, 서버가 한계값 클램프와 속도 제한을
  적용해 /arm_controller, /gripper_controller 액션으로 전달한다. SIGUSR1을 받으면 제어를 잠근다(종료 중).
- /term 웹소켓은 pty로 bash를 띄워 xterm.js와 연결한다. (사실상 로컬 셸이므로
  127.0.0.1에만 바인딩하고, 랜덤 토큰 + Origin 검사를 한다.)
사용: python3 dashboard.py [--port 8765] [--topic /camera1/image_raw]  (env.sh를 source한 셸에서)
시작하면 접속 URL(토큰 포함)을 stdout에 한 줄로 출력한다: DASHBOARD_URL=...
"""
import argparse
import asyncio
import fcntl
import json
import math
import os
import pty
import secrets
import signal
import struct
import termios
import threading
import time
from http import HTTPStatus
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from websockets.asyncio.server import serve
from websockets.datastructures import Headers
from websockets.http11 import Response

HERE = Path(__file__).resolve().parent
JPEG_FPS = 30      # 인코딩 상한. 카메라가 이보다 느리면 들어오는 만큼만 보낸다
JOINT_HZ = 15      # 관절 상태 전송 주기
POLL_S = 0.005     # 새 영상 프레임 확인 주기 (프레임이 오면 최대 5ms 안에 전송)
VIDEO_MAX_W = 640

# 관절 한계 (URDF open_manipulator_x_arm.urdf.xacro 기준, rad / 그리퍼는 m). 서버에서 한 번 더 클램프한다.
ARM = ["joint1", "joint2", "joint3", "joint4"]
LIMITS = {"joint1": (-3.14, 3.14), "joint2": (-1.5, 1.5), "joint3": (-1.5, 1.4), "joint4": (-1.7, 1.97),
          "gripper_left_joint": (-0.011, 0.02)}
MAX_SPEED = 0.6   # rad/s, 가장 많이 움직이는 관절 기준 (go_home 0.35, dance 1.2 사이)
MIN_TIME = 1.0


class Bridge(Node):
    def __init__(self, topic):
        super().__init__("mr_crab_dashboard")
        self.lock = threading.Lock()
        self.joints = {}          # name -> (pos, effort)
        self.jpeg = None
        self.jpeg_seq = 0
        self.want_video = False   # 보는 클라이언트가 있을 때만 JPEG 인코딩
        self._last_enc = 0.0
        self.locked = False       # True면 대시보드 제어 거부 (bringup 종료 중 홈 이동과 충돌 방지)
        self.arm_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.hand_ts = 0.0        # hand_follow.py가 그린 영상(/camera1/image_hand)이 최근에 왔으면 그걸 우선 표시
        self.create_subscription(Image, topic, lambda m: self.on_image(m, False), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera1/image_hand", lambda m: self.on_image(m, True), qos_profile_sensor_data)

    def on_joints(self, msg):
        eff = list(msg.effort) + [0.0] * (len(msg.name) - len(msg.effort))
        with self.lock:
            for n, p, e in zip(msg.name, msg.position, eff):
                self.joints[n] = (p, e)

    # ── 제어 ── 각 함수는 (성공 여부, 메시지)를 돌려준다
    def _clamp(self, name, v):
        lo, hi = LIMITS[name]
        return min(max(float(v), lo), hi)

    def move_arm(self, target, clamp=True):
        """target: {joint: rad}. 빠진 관절은 현재 위치 그대로(클램프 안 함: 홈 자세는 joint2 한계 밖이다).
        clamp=False는 신뢰하는 home.json 전용. 새 목표는 진행 중인 목표를 대체한다."""
        if self.locked:
            return False, "종료 중이라 제어가 잠겼습니다"
        with self.lock:
            cur = {n: self.joints[n][0] for n in ARM if n in self.joints}
        if len(cur) < len(ARM):
            return False, "/joint_states를 아직 받지 못했습니다"
        if not self.arm_client.wait_for_server(timeout_sec=0.0):
            return False, "arm_controller 액션 서버가 없습니다"
        goal_pos = [(self._clamp(n, target[n]) if clamp else float(target[n])) if n in target else cur[n] for n in ARM]
        if not all(math.isfinite(g) for g in goal_pos):   # NaN 은 클램프를 그대로 통과하므로 따로 막는다
            return False, "잘못된 관절 값입니다"
        delta = max(abs(g - cur[n]) for g, n in zip(goal_pos, ARM))
        if delta < 0.005:
            return True, "이미 목표 위치입니다"
        T = max(delta / MAX_SPEED, MIN_TIME)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = goal_pos
        pt.time_from_start = Duration(sec=int(T), nanosec=int((T % 1) * 1e9))
        goal.trajectory.points = [pt]
        self.arm_client.send_goal_async(goal)
        return True, f"이동 {T:.1f}초"

    def move_gripper(self, pos):
        if self.locked:
            return False, "종료 중이라 제어가 잠겼습니다"
        if not math.isfinite(pos):
            return False, "잘못된 그리퍼 값입니다"
        if not self.grip_client.wait_for_server(timeout_sec=0.0):
            return False, "gripper_controller 액션 서버가 없습니다"
        g = GripperCommand.Goal()
        g.command.position = self._clamp("gripper_left_joint", pos)
        g.command.max_effort = 0.0   # dance.py와 같은 값
        self.grip_client.send_goal_async(g)
        return True, "그리퍼 이동"

    def go_home(self):
        try:
            home = json.load(open(HERE / "home.json"))
        except Exception as e:
            return False, f"home.json을 읽지 못했습니다: {e}"
        return self.move_arm({n: home[n] for n in ARM if n in home}, clamp=False)

    def stop_arm(self):
        """현재 위치를 새 목표로 보내 진행 중인 이동을 멈춘다 (잠금 중에도 허용)."""
        with self.lock:
            cur = {n: self.joints[n][0] for n in ARM if n in self.joints}
        if len(cur) < len(ARM) or not self.arm_client.wait_for_server(timeout_sec=0.0):
            return False, "정지 명령을 보낼 수 없습니다"
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [cur[n] for n in ARM]
        pt.time_from_start = Duration(sec=0, nanosec=300_000_000)
        goal.trajectory.points = [pt]
        self.arm_client.send_goal_async(goal)
        return True, "정지"

    def on_image(self, msg, hand=False):
        now = time.monotonic()
        if hand:
            self.hand_ts = now
        elif now - self.hand_ts < 1.0:
            return
        if not self.want_video or now - self._last_enc < 1.0 / JPEG_FPS:
            return
        self._last_enc = now
        enc = msg.encoding
        try:
            if enc in ("rgb8", "bgr8"):
                a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
                a = a.reshape(msg.height, msg.width, 3)
                bgr = a[:, :, ::-1] if enc == "rgb8" else a
            elif enc == "mono8":
                a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
                bgr = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
            else:
                return
            if bgr.shape[1] > VIDEO_MAX_W:
                s = VIDEO_MAX_W / bgr.shape[1]
                bgr = cv2.resize(bgr, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(bgr), [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()
                    self.jpeg_seq += 1
        except Exception as e:  # 한 프레임 실패는 무시
            self.get_logger().warn(f"image encode failed: {e}", throttle_duration_sec=5.0)


def make_app(bridge, token, port):
    clients = set()
    allowed_origins = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
    index_html = (HERE / "dashboard.html").read_bytes()

    def text_response(status, body, ctype="text/plain; charset=utf-8"):
        h = Headers([("Content-Type", ctype), ("Content-Length", str(len(body))), ("Cache-Control", "no-store")])
        return Response(status.value, status.phrase, h, body)

    async def process_request(conn, request):
        u = urlparse(request.path)
        q = parse_qs(u.query)
        ok_token = secrets.compare_digest(q.get("t", [""])[0], token)
        if u.path in ("/ws", "/term"):
            # 웹소켓 업그레이드: 토큰과 Origin이 맞아야만 허용 (다른 웹사이트의 접속 차단)
            if not ok_token or request.headers.get("Origin") not in allowed_origins:
                return text_response(HTTPStatus.FORBIDDEN, b"forbidden\n")
            return None
        if u.path == "/":
            if not ok_token:
                return text_response(HTTPStatus.FORBIDDEN, b"forbidden (token)\n")
            return text_response(HTTPStatus.OK, index_html, "text/html; charset=utf-8")
        return text_response(HTTPStatus.NOT_FOUND, b"not found\n")

    async def ws_handler(ws):
        if ws.request.path.startswith("/term"):
            return await term_handler(ws)
        clients.add(ws)
        bridge.want_video = True
        seq = 0
        last_cmd = 0.0
        next_joint = 0.0

        async def recv_cmds():
            nonlocal last_cmd
            async for raw in ws:
                if not isinstance(raw, str):
                    continue
                try:
                    m = json.loads(raw)
                    cmd = m["cmd"]
                    now = time.monotonic()
                    if cmd != "stop" and now - last_cmd < 0.05:   # 명령 폭주 방지
                        continue
                    last_cmd = now
                    if cmd == "move":
                        ok, text = bridge.move_arm({k: float(v) for k, v in m["joints"].items() if k in ARM})
                    elif cmd == "gripper":
                        ok, text = bridge.move_gripper(float(m["position"]))
                    elif cmd == "home":
                        ok, text = bridge.go_home()
                    elif cmd == "stop":
                        ok, text = bridge.stop_arm()
                    else:
                        continue
                except (KeyError, ValueError, TypeError, AttributeError):
                    ok, text = False, "잘못된 명령"
                await ws.send(json.dumps({"result": {"ok": ok, "msg": text}}))

        recv_task = asyncio.create_task(recv_cmds())
        try:
            while True:
                with bridge.lock:
                    jpeg, jseq = bridge.jpeg, bridge.jpeg_seq
                if jpeg is not None and jseq != seq:   # 영상은 새 프레임이 오는 즉시 (폴링 주기에 묶이지 않게)
                    seq = jseq
                    await ws.send(jpeg)
                now = time.monotonic()
                if now >= next_joint:                   # 관절 상태는 JOINT_HZ로
                    next_joint = now + 1.0 / JOINT_HZ
                    with bridge.lock:
                        joints = dict(bridge.joints)
                    await ws.send(json.dumps({"joints": {n: [p, e] for n, (p, e) in joints.items()},
                                              "locked": bridge.locked}))
                await asyncio.sleep(POLL_S)
        except Exception:
            pass
        finally:
            recv_task.cancel()
            clients.discard(ws)
            if not clients:
                bridge.want_video = False

    async def term_handler(ws):
        pid, fd = pty.fork()
        if pid == 0:  # 자식: 대화형 bash (dashboard_rc.sh로 env.sh 적용)
            os.chdir(str(HERE))
            os.environ["TERM"] = "xterm-256color"
            os.execvp("bash", ["bash", "--rcfile", str(HERE / "dashboard_rc.sh"), "-i"])
        loop = asyncio.get_running_loop()
        q = asyncio.Queue()

        def on_readable():
            try:
                data = os.read(fd, 65536)
            except OSError:
                data = b""
            q.put_nowait(data)
            if not data:
                loop.remove_reader(fd)

        loop.add_reader(fd, on_readable)

        async def pump_out():
            while True:
                data = await q.get()
                if not data:
                    await ws.close()
                    return
                await ws.send(data)

        out_task = asyncio.create_task(pump_out())
        try:
            async for msg in ws:
                if isinstance(msg, str):
                    try:
                        m = json.loads(msg)
                    except ValueError:
                        m = None
                    if isinstance(m, dict) and "resize" in m:
                        cols, rows = (int(x) for x in m["resize"])
                        fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
                    else:
                        os.write(fd, msg.encode())
                else:
                    os.write(fd, msg)
        except Exception:
            pass
        finally:
            out_task.cancel()
            try:
                loop.remove_reader(fd)
            except Exception:
                pass
            # bash 를 끝내고 회수한다. waitpid(pid, 0) 으로 막고 기다리면 bash 가 안 끝날 때 대시보드 전체가 멈추므로
            # 이벤트 루프를 막지 않게 짧게 나눠 확인하고, 2초 안에 안 끝나면 강제 종료한다.
            try:
                os.kill(pid, signal.SIGHUP)
                for _ in range(20):
                    if os.waitpid(pid, os.WNOHANG)[0]:
                        break
                    await asyncio.sleep(0.1)
                else:
                    os.kill(pid, signal.SIGKILL)
                    os.waitpid(pid, 0)
            except Exception:
                pass
            try:
                os.close(fd)
            except OSError:
                pass

    return ws_handler, process_request


async def main(args):
    rclpy.init()
    bridge = Bridge(args.topic)
    threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True).start()
    token = secrets.token_urlsafe(16)
    handler, process_request = make_app(bridge, token, args.port)
    async with serve(handler, "127.0.0.1", args.port, process_request=process_request,
                     max_size=2 ** 20, compression=None):
        print(f"DASHBOARD_URL=http://127.0.0.1:{args.port}/?t={token}", flush=True)
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for s in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(s, stop.set)
        loop.add_signal_handler(signal.SIGUSR1, lambda: setattr(bridge, "locked", True))
        await stop.wait()
    rclpy.shutdown()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--topic", default="/camera1/image_raw")
    asyncio.run(main(ap.parse_args()))
