#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
 E0509 Vision Pick & Place (Real Robot & Gazebo Simulation Compatible)
=====================================================================
카메라 창에서 물체 클릭 → 핸드-아이 결과 + 오프셋으로 base 좌표 계산
→ 현재 높이 유지한 채 물체 위로 XY 정렬 → 터미널 메뉴로 하강/파지/놓기.

실물 로봇(RealSense USB)과 Gazebo 시뮬레이션 환경 모두를 100% 지원합니다.
- 실물 카메라 미연결 또는 --sim 옵션 시 자동으로 Gazebo 시뮬레이션 카메라로 전환
- ~/HamdEyeCal 설정 파일 부재 시 Gazebo URDF 정밀 캘리브레이션 행렬 자동 적용

실행:
  python3 vision_pick_place.py
  python3 vision_pick_place.py --sim
  python3 vision_pick_place.py --place place
=====================================================================
"""

import os
import sys
import json
import time
import argparse
import threading
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# ── ROS 2 및 기본 DSR 초기화 ─────────────────────────────
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("vision_pick_place", namespace=ROBOT_ID)
DR_init.__dsr__node = node

import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, movejx, posj, posx, get_current_posx,  # noqa: E402
                        get_robot_mode, check_motion, DR_BASE, DR_MV_MOD_ABS)
from dsr_gripper import gripper_open, gripper_close  # noqa: E402

# ── 인자 파싱 및 기본 경로 설정 ───────────────────────────
CONFIG_PATH = Path("~/HamdEyeCal/handeye_config.json").expanduser()
POSES_PATH = Path(__file__).resolve().parent / "poses.json"
READY = (0, 0, 90, 0, 90, 0)
VEL = 20                                   # 저속 고정 (mm/s, deg/s)
REACH_MIN, REACH_MAX = 150.0, 850.0        # 수평 도달 반경(mm)
DEPTH_MIN, DEPTH_MAX = 0.2, 1.5            # 유효 깊이(m)
WIN = "click object: SPACE=confirm, q=quit"
GRIP_WAIT = 1.5                            # 그리퍼 명령 후 다음 이동까지 대기(초)

_ap = argparse.ArgumentParser(description="E0509 비전 픽앤플레이스")
_ap.add_argument("--place", default="place", help="poses.json 의 놓을 자세 이름")
_ap.add_argument("--sim", action="store_true", help="Gazebo 시뮬레이션 카메라 강제 사용")
ARGS = _ap.parse_args()

# ── 캘리브레이션 및 오프셋 로드 ───────────────────────────
# Gazebo Table Simulation Default Transform (URDF Camera Stand):
# Camera Lens Center in Robot Base: X=755mm, Y=-50mm, Z=905mm, pointing straight down (-Z)
T_BASE_CAM_SIM = np.array([
    [0.0,  1.0,  0.0, 0.755],
    [1.0,  0.0,  0.0, -0.050],
    [0.0,  0.0, -1.0, 0.905],
    [0.0,  0.0,  0.0, 1.0]
])

is_sim_mode = ARGS.sim

if not is_sim_mode and CONFIG_PATH.exists():
    try:
        CFG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        CALIB = CFG.get("calib_path") or str(Path("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy").expanduser())
        T_BASE_CAM = np.load(CALIB)
        OFFSET = np.array([CFG.get("offset_x_mm", 0.0), CFG.get("offset_y_mm", 0.0), CFG.get("offset_z_mm", 0.0)])
        TCP_Z = float(CFG.get("tcp_z_mm", 200.0))
        STEP = float(CFG.get("step_mm", 10.0))
        Z_MIN = float(CFG.get("z_min_mm", 50.0))
        print(f"[실물 모드] 캘리브 로드: {CALIB}")
    except Exception as e:
        print(f"[경고] 설정 로드 실패 ({e}) → 시뮬레이션 기본값 적용")
        is_sim_mode = True
else:
    is_sim_mode = True

if is_sim_mode:
    T_BASE_CAM = T_BASE_CAM_SIM
    OFFSET = np.array([0.0, 0.0, 0.0])
    TCP_Z = 116.5  # RH-P12-RN fingertip distance
    STEP = 15.0
    Z_MIN = 20.0
    CFG = {}
    print("=" * 65)
    print(" [비전 픽앤플레이스] 시뮬레이션 환경 모드로 동작합니다.")
    print("   카메라 입력: Gazebo /camera/image & /camera/depth_image")
    print(f"   기구학 기준 캘리브레이션 행렬 자동 적용 (Z_MIN={Z_MIN:.0f}mm, TCP={TCP_Z:.1f}mm)")
    print("=" * 65)

PLACE = json.loads(POSES_PATH.read_text()).get(ARGS.place) if POSES_PATH.exists() else None
if not PLACE:
    # 시뮬레이션 기본 place 자세 (스태킹 위치) 제공
    PLACE = {
        "posj": [15.119, 31.514, 77.447, 1.088, 69.197, -126.634],
        "posx": [533.337, 147.262, 228.91, 43.985, 177.893, -97.397],
        "sol": 2
    }
    print(f"[안내] poses.json에 '{ARGS.place}' 없음 → 기본 place 포즈 사용: {PLACE['posx'][:3]}")
else:
    print(f"place 자세 로드 완료 ('{ARGS.place}'): posx={PLACE['posx'][:3]}")


# ── Gazebo 시뮬레이션 카메라 어댑터 ──────────────────────────
class SimDepthFrame:
    def __init__(self, depth_img, intrinsics):
        self.depth_img = depth_img
        self.intrinsics = intrinsics

    def get_distance(self, u, v):
        h, w = self.depth_img.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            val = float(self.depth_img[int(v), int(u)])
            return val
        return 0.0


class SimFrames:
    def __init__(self, color_img, depth_img, intrinsics):
        self.color_img = color_img
        self.depth_frame = SimDepthFrame(depth_img, intrinsics)

    def get_color_frame(self):
        return self

    def get_depth_frame(self):
        return self.depth_frame

    def get_data(self):
        return self.color_img


class SimCameraPipeline:
    def __init__(self, ros_node):
        self.node = ros_node
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color_img = None
        self.depth_img = None
        self.intrinsics = {
            'width': 640, 'height': 480,
            'fx': 381.39, 'fy': 381.39,
            'cx': 320.0, 'cy': 240.0
        }

        self.sub_color = self.node.create_subscription(
            Image, '/camera/image', self._color_cb, 10
        )
        self.sub_depth = self.node.create_subscription(
            Image, '/camera/depth_image', self._depth_cb, 10
        )
        self.sub_info = self.node.create_subscription(
            CameraInfo, '/camera/camera_info', self._info_cb, 10
        )

    def _color_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.color_img = cv_img
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.lock:
                self.depth_img = np.nan_to_num(cv_depth, nan=0.0, posinf=0.0, neginf=0.0)
        except Exception:
            pass

    def _info_cb(self, msg):
        if msg.k[0] > 0:
            self.intrinsics = {
                'width': msg.width, 'height': msg.height,
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5]
            }

    def wait_for_frames(self, timeout_sec=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            with self.lock:
                if self.color_img is not None and self.depth_img is not None:
                    return SimFrames(self.color_img.copy(), self.depth_img.copy(), self.intrinsics)
        return None

    def stop(self):
        pass


# ── 카메라 시작 함수 (실물 / 시뮬레이션 분기) ────────────────
def start_camera():
    global is_sim_mode
    if not is_sim_mode:
        try:
            import pyrealsense2 as rs
            pipe = rs.pipeline()
            for (cw, ch, cf), (dw, dh, df) in [((1280, 720, 30), (1280, 720, 30)),
                                               ((1280, 720, 15), (848, 480, 15)),
                                               ((640, 480, 30), (640, 480, 30))]:
                try:
                    cfg = rs.config()
                    if CFG.get("camera_serial"):
                        cfg.enable_device(CFG["camera_serial"])
                    cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cf)
                    cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, df)
                    pipe.start(cfg)
                    print(f"카메라: color {cw}x{ch}@{cf}")
                    break
                except RuntimeError:
                    continue
            else:
                raise RuntimeError("USB RealSense 연결 실패")
            align = rs.align(rs.stream.color)
            for _ in range(15):
                align.process(pipe.wait_for_frames())
            return pipe, align
        except Exception as e:
            print(f"[알림] 실물 RealSense 감지 안됨 ({e}) → Gazebo 가상 카메라로 전환합니다.")
            is_sim_mode = True

    # Gazebo 시뮬레이션 카메라 연결
    print("[비전] Gazebo 카메라 스트림 연결 대기 중 (/camera/image)...")
    sim_cam = SimCameraPipeline(node)
    f = sim_cam.wait_for_frames(timeout_sec=5.0)
    if f is None:
        raise SystemExit("Gazebo 카메라 토픽을 수신하지 못했습니다. 시뮬레이션이 실행 중인지 확인하세요.")
    print("[비전] Gazebo 카메라 스트림 연결 완료!")
    return sim_cam, None


# ── 3D 좌표 변환 ──────────────────────────────────────────
def pixel_to_cam(u, v, depth_frame, win=4):
    """(u, v) 주변 깊이 중앙값으로 카메라 좌표 3D 점(m)."""
    pts = []
    intr = getattr(depth_frame, 'intrinsics', None)
    if intr is None:
        try:
            import pyrealsense2 as rs
            intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
            w, h = intr.width, intr.height
            deproject = lambda u, v, z: rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], z)
        except Exception:
            return None
    else:
        w, h = intr['width'], intr['height']
        deproject = lambda u, v, z: [
            (float(u) - intr['cx']) * z / intr['fx'],
            (float(v) - intr['cy']) * z / intr['fy'],
            z
        ]

    for du in range(-win, win + 1):
        for dv in range(-win, win + 1):
            uu, vv = u + du, v + dv
            if 0 <= uu < w and 0 <= vv < h:
                z = depth_frame.get_distance(uu, vv)
                if DEPTH_MIN <= z <= DEPTH_MAX:
                    pts.append(deproject(uu, vv, z))
    return np.median(np.array(pts), axis=0) if pts else None


def cam_to_base_mm(p_cam):
    return (T_BASE_CAM @ np.append(p_cam, 1.0))[:3] * 1000.0 + OFFSET


def pick_target(pipe, align):
    """라이브 창에서 클릭 → SPACE 로 확정. 반환: base 좌표(mm) 또는 None(q)."""
    state = {"uv": None}
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, lambda e, x, y, *_: state.update(uv=(x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    target = None

    while True:
        if align is not None:
            f = align.process(pipe.wait_for_frames())
        else:
            f = pipe.wait_for_frames()

        if f is None:
            time.sleep(0.02)
            continue

        color = np.asanyarray(f.get_color_frame().get_data()).copy()
        depth = f.get_depth_frame()

        if state["uv"]:
            p = pixel_to_cam(*state["uv"], depth)
            target = cam_to_base_mm(p) if p is not None else None
            cv2.drawMarker(color, state["uv"], (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
            txt = (f"base x={target[0]:.0f} y={target[1]:.0f} z={target[2]:.0f} mm"
                   if target is not None else "no valid depth - click elsewhere")
            cv2.putText(color, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow(WIN, color)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            target = None
            break
        if k in (ord(" "), 13) and target is not None:
            r = float(np.hypot(*target[:2]))
            if REACH_MIN < r < REACH_MAX:
                break
            print(f"수평거리 {r:.0f}mm — 도달범위 밖, 다른 곳 클릭")

    cv2.destroyWindow(WIN)
    cv2.waitKey(1)
    return target


# ── 로봇 제어 헬퍼 함수 ────────────────────────────────────
def wait_idle(timeout=30.0):
    """check_motion()==0(IDLE)이 될 때까지 대기."""
    t0 = time.time()
    time.sleep(0.2)
    while check_motion() != 0:
        if time.time() - t0 > timeout:
            raise RuntimeError("이동 시간 초과 — 정지합니다.")
        time.sleep(0.1)
    time.sleep(0.2)


def tip_now():
    pose, _ = get_current_posx()
    R = Rot.from_euler("ZYZ", pose[3:6], degrees=True).as_matrix()
    return np.array(pose[:3]) + R @ [0, 0, TCP_Z], list(pose)


def move_tip(xyz, rxyz):
    R = Rot.from_euler("ZYZ", rxyz, degrees=True).as_matrix()
    f = np.array(xyz) - R @ [0, 0, TCP_Z]
    movel(posx(*map(float, f), *map(float, rxyz)), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    wait_idle()


def go_place(safe_z):
    """현재 높이로 들어올린 뒤 place 위(safe 높이) → 가르친 자세로 하강 → 열기 → 상승."""
    tip, pose = tip_now()
    move_tip([tip[0], tip[1], safe_z], pose[3:6])
    above = list(PLACE["posx"]); above[2] += 100
    movejx(posx(above), vel=VEL, acc=VEL, ref=DR_BASE, sol=PLACE["sol"])
    wait_idle()
    movel(posx(PLACE["posx"]), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    wait_idle()
    gripper_open(); time.sleep(GRIP_WAIT)
    movel(posx(above), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS)


MENU = ("\n[Enter] 한 스텝 내리기  a) 물체 윗면 +20mm 까지  g) 잡기  o) 열기\n"
        "u) 처음 높이로 올리기  p) place 로 옮겨 놓기  r) 준비자세·다음 물체  q) 종료\n> ")


def main():
    pipe = None
    try:
        assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), \
            "로봇 서비스 없음 — bringup(또는 sim_bridge.py) 실행 여부 확인"
        assert get_robot_mode() == 1, "auto 모드가 아닙니다"

        pipe, align = start_camera()

        while True:
            target = pick_target(pipe, align)
            if target is None:
                break

            print(f"\n🎯 목표(base) x={target[0]:.0f} y={target[1]:.0f} z={target[2]:.0f} mm")
            input("⚠️  준비자세 → 물체 위로 이동합니다. 주변 확인 후 Enter (중단 Ctrl+C) ")

            movej(posj(*READY), vel=VEL, acc=VEL)
            wait_idle()
            gripper_open(); time.sleep(GRIP_WAIT)

            tip, pose = tip_now()
            safe_z = tip[2]
            move_tip([target[0], target[1], safe_z], pose[3:6])
            print("물체 위 정렬 완료 — 그리퍼 끝이 물체 바로 위인지 확인하세요.")

            while True:
                c = input(MENU).strip().lower()
                tip, pose = tip_now()
                if c in ("", "a"):
                    z = tip[2] - STEP if c == "" else target[2] + 20
                    if z < Z_MIN:
                        print(f"거부: 그리퍼 끝 {z:.0f}mm < 최저 {Z_MIN:.0f}mm")
                        continue
                    move_tip([tip[0], tip[1], z], pose[3:6])
                    print(f"그리퍼 끝 z≈{z:.0f} | 물체 윗면 z≈{target[2]:.0f}")
                elif c == "g":
                    wait_idle(); gripper_close(current=300); time.sleep(GRIP_WAIT)
                elif c == "o":
                    wait_idle(); gripper_open(); time.sleep(GRIP_WAIT)
                elif c == "u":
                    move_tip([tip[0], tip[1], safe_z], pose[3:6])
                elif c == "p":
                    go_place(safe_z)
                    print("놓기 완료")
                elif c == "r":
                    move_tip([tip[0], tip[1], safe_z], pose[3:6])
                    movej(posj(*READY), vel=VEL, acc=VEL)
                    wait_idle()
                    break
                elif c == "q":
                    raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        if pipe:
            pipe.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
