#!/usr/bin/env python3
"""손 따라 움직이기 (참참참): 고정된 카메라로 손의 위치를 보고, 손이 좌우/상하로 움직이는 대로 로봇팔이 좌우/상하로 움직인다.

hand_follow.py(카메라가 팔에 달려 손을 화면 중앙에 맞추는 방식)와 달리 카메라는 고정이고, 손의 화면 위치를 팔 자세에 직접 대응시킨다.
카메라 지연이 제어 루프 안에 들어가지 않아 진동 위험이 작다.

- 손이 화면 가운데(--neutral-x/y) 에 있으면 팔은 기준 자세(--pose-file, 기본 teleop_pose.json: 위아래 ±13cm 범위가 대칭이 되도록 고른 자세).
- 손이 화면 좌우로 움직이면 joint1(좌우 회전), 위아래로 움직이면 끝점 높이가 바뀐다. 높이는 평면 역기구학으로 joint2·3을 풀고
  joint4가 기울기(그리퍼 방향)를 그대로 유지한다. 손 위치는 필터링하고, 팔 속도/관절 한계/최소 높이를 제한한다.
- 제스처는 hand_follow와 같다: 주먹=그리퍼 닫기, 폄=열기, 중지·약지 접고 새끼 폄=그 자리 정지(풀면 다시 따라감), 검지만 폄=기준 자세로 이동.
사용: ./hand_teleop.sh [--dry-run] [--topic /camera/camera/color/image_raw] [--pan-sign 1] [--z-sign -1] ...
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

import hand_follow as hf
from hand_follow import ARM, HERE, JLIM, HandFollow, fk, run, slew


def solve_ik(p_target, q2, q3, iters=40):
    """joint4 위치(p4)를 p_target(x, z)에 놓는 (q2, q3). 감쇠 최소제곱 반복. 반환: (q2, q3, 수렴 여부)."""
    e, lam = 1e-6, 0.01
    for _ in range(iters):
        p = np.array(fk(q2, q3, 0.0)[0])
        err = np.asarray(p_target) - p
        if np.linalg.norm(err) < 1e-4:
            return q2, q3, True
        c2 = (np.array(fk(q2 + e, q3, 0.0)[0]) - p) / e
        c3 = (np.array(fk(q2, q3 + e, 0.0)[0]) - p) / e
        J = np.array([[c2[0], c3[0]], [c2[1], c3[1]]])
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(2), err)
        q2, q3 = q2 + float(dq[0]), q3 + float(dq[1])
    return q2, q3, bool(np.linalg.norm(np.asarray(p_target) - np.array(fk(q2, q3, 0.0)[0])) < 1e-3)


class HandTeleop(HandFollow):
    def __init__(self, a):
        super().__init__(a)
        if self.home is None:
            sys.exit(f"기준 자세 파일을 읽을 수 없습니다: {a.pose_file}")
        h = self.home
        self.q0 = {j: float(h[j]) for j in ARM}                       # 손이 가운데일 때의 팔 자세
        self.phi0 = self.q0["joint2"] + self.q0["joint3"] + self.q0["joint4"]   # 유지할 그리퍼 피치
        self.p4_0 = np.array(fk(self.q0["joint2"], self.q0["joint3"], self.q0["joint4"])[0])

    @staticmethod
    def _dz(v, dead):                      # 데드존 밖 초과분만 사용하고 ±1로 제한
        v = 0.0 if abs(v) < dead else v - math.copysign(dead, v)
        return max(-1.0, min(1.0, v / (1.0 - dead)))

    def map_step(self, center, dt):
        """손 위치(정규화 화면 좌표) → 팔 관절 목표를 vmax로 따라가게 갱신."""
        a = self.a
        ex = self._dz((center[0] - a.neutral_x) / 0.5, a.deadzone)        # +: 손이 화면 오른쪽
        ey = self._dz((a.neutral_y - center[1]) / 0.5, a.deadzone)        # +: 손이 위쪽
        t = self.target
        # 좌우: joint1 (절대 대응)
        j1 = self.q0["joint1"] + a.pan_sign * ex * a.yaw_range
        j1 = min(max(j1, self.q0["joint1"] - a.pan_range), self.q0["joint1"] + a.pan_range)
        # 상하: 끝점(joint4 위치) 높이 + 피치 유지. 한계/최소 높이를 넘으면 그 범위 안에서 가장 큰 값을 쓴다
        dz = a.z_sign * ey * a.z_range

        def try_dz(d):   # d만큼 올렸을 때의 관절 해. 한계/최소 높이를 넘거나 못 풀면 None
            q2, q3, ok = solve_ik(self.p4_0 + np.array([0.0, d]), t["joint2"], t["joint3"])
            cand = {"joint2": q2, "joint3": q3, "joint4": self.phi0 - q2 - q3}
            p4, tip = fk(cand["joint2"], cand["joint3"], cand["joint4"])
            if not ok or any(not (JLIM[j][0] <= v <= JLIM[j][1]) for j, v in cand.items()) or min(p4[1], tip[1]) < a.min_height:
                return None
            return cand
        sol, used = try_dz(dz), dz
        if sol is None:   # 범위를 넘으면 가능한 가장 큰 값까지 이분 탐색 (한계에서 값이 계단식으로 튀지 않게)
            lo, hi, sol = 0.0, 1.0, try_dz(0.0)
            for _ in range(8):
                mid = (lo + hi) / 2
                c = try_dz(dz * mid)
                if c is not None:
                    lo, sol = mid, c
                else:
                    hi = mid
            used = dz * lo
        if sol is None:
            sol = {j: t[j] for j in ("joint2", "joint3", "joint4")}   # 풀 수 없으면 그대로
        sol["joint1"] = j1
        vmax, lead = a.max_speed, a.max_speed * 0.6
        # 속도 제한: 모든 관절을 같은 비율로 줄여서 동시에 도착하게 한다 (관절별로 따로 제한하면 j2+j3+j4(피치)가 이동 중 틀어짐)
        steps = {j: sol[j] - t[j] for j in ARM}
        big = max(abs(v) for v in steps.values())
        k = 1.0 if big <= vmax * dt else vmax * dt / big
        for j in ARM:   # + 안티 와인드업(실제 자세에서 너무 앞서가지 않게)
            t[j] = min(max(t[j] + k * steps[j], self.cur[j] - lead), self.cur[j] + lead)
        limited = "" if abs(used - dz) < 1e-6 else " (한계)"
        self.depth_note = f"yaw {math.degrees(j1 - self.q0['joint1']):+.0f}° z {used * 100:+.1f}cm{limited}"

    def control(self, raw_center, raw_size, now, dt, absent, hold=False):
        if absent:
            for f in (self.fx, self.fy, self.fs):
                f.reset()
        if self.a.smooth > 0:
            center = (self.fx(raw_center[0], now), self.fy(raw_center[1], now))
        else:
            center = raw_center
        size = raw_size
        if self.homing:   # 기준 자세로 이동 중에는 목표/명령을 건드리지 않는다
            return center, size, True
        if self.target is None or absent:   # 처음/다시 보일 때 현재 자세에서 시작해 매핑된 자세로 부드럽게 이동
            if not all(j in self.cur for j in ARM):
                return center, size, False
            self.target = {j: self.cur[j] for j in ARM}
        if not hold:
            self.map_step(center, dt)
        published = self.maybe_publish(now)
        if self.log:
            self.log.write(",".join(f"{v:.5f}" for v in (now, *raw_center, raw_size, *center, size, 0.0, 0.0, 0.0,
                           *[self.target[j] for j in ARM], *[self.cur[j] for j in ARM])) + f",{int(published)}\n")
        return center, size, True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/camera/camera/color/image_raw", help="컬러 영상 토픽 (RealSense 기본값). C270이면 /camera1/image_raw")
    ap.add_argument("--model", type=Path, default=HERE / "models" / "hand_landmarker.task")
    ap.add_argument("--dry-run", action="store_true", help="로봇에는 명령을 보내지 않고 인식/계산 결과만 보여 준다")
    ap.add_argument("--no-gripper", action="store_true")
    ap.add_argument("--pose-file", type=Path, default=HERE / "teleop_pose.json", help="손이 가운데일 때의 팔 자세(위아래 범위가 대칭인 자세). 검지 제스처의 목표 자세이기도 함")
    ap.add_argument("--neutral-x", type=float, default=0.5, help="이 화면 위치(0~1)에 손이 있을 때 기준 자세")
    ap.add_argument("--neutral-y", type=float, default=0.5)
    ap.add_argument("--deadzone", type=float, default=0.05, help="가운데 허용 오차 (±1 범위 대비 비율)")
    ap.add_argument("--yaw-range", type=float, default=0.8, help="손이 화면 끝에 있을 때의 좌우 회전각 (rad)")
    ap.add_argument("--pan-range", type=float, default=1.2, help="기준 자세에서 좌우로 갈 수 있는 최대각 (rad)")
    ap.add_argument("--pan-sign", type=float, default=-1.0, help="손이 화면 오른쪽일 때 joint1 방향. 반대면 1 (카메라가 로봇과 같은 방향을 볼 때 -1)")
    ap.add_argument("--z-range", type=float, default=0.08, help="손이 화면 위/아래 끝일 때 끝점 높이 변화 (m)")
    ap.add_argument("--z-sign", type=float, default=1.0, help="손을 올렸을 때 팔이 올라가면 1. 반대면 -1")
    ap.add_argument("--min-height", type=float, default=0.05, help="joint4/끝점 최소 높이 (m, 베이스 기준)")
    ap.add_argument("--max-speed", type=float, default=0.5, help="관절 최대 속도 (rad/s)")
    ap.add_argument("--smooth", type=float, default=1.5, help="손 위치 필터 최소 차단주파수(Hz). 클수록 빠르게 반응, 작을수록 부드러움")
    ap.add_argument("--smooth-beta", type=float, default=8.0, help="빠르게 움직일 때 필터를 풀어 주는 정도")
    ap.add_argument("--pub-rate", type=float, default=30.0)
    ap.add_argument("--pub-eps", type=float, default=0.001)
    ap.add_argument("--horizon", type=float, default=0.05)
    ap.add_argument("--home-confirm", type=int, default=10, help="검지 제스처 인정 연속 프레임 수")
    ap.add_argument("--home-speed", type=float, default=hf.HOME_SPEED)
    ap.add_argument("--confirm", type=int, default=4, help="주먹/펴기/정지 인정 연속 프레임 수")
    ap.add_argument("--log", default=None)
    a = ap.parse_args()
    a.no_depth, a.ref_size = True, None   # HandFollow가 요구하는 값 (여기서는 크기 추적을 쓰지 않음)
    run(a, HandTeleop, f"카메라 고정 텔레오퍼레이션 yaw*{a.pan_sign:+.0f} z*{a.z_sign:+.0f} (손이 {a.neutral_x:.1f},{a.neutral_y:.1f}이면 기준 자세)")


if __name__ == "__main__":
    main()
