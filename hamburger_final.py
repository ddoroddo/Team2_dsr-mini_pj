#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
  E0509 Burger Assembly & Autonomous N-Tier Stacking System
=====================================================================
  기능:
  1. YOLOv8 실시간 비전 인식 (5종 재료: Bun Top, Bottom Bun, Patty, Tomato, Cheese)
  2. OpenCV 기반 박스 정밀 회전각(Yaw) 추출 및 J6 사전 정렬 회전
  3. 사용자 지정 순서(레시피) 기반 임의 위치 N단 정밀 적재
  4. 사전 기구학(IK/FK) 정밀 도달성 검증으로 로봇 알람(1206/1207) 원천 방지
  5. 책상 충돌 방지(-28mm) 및 누적 높이 기반 계단식 수직 접근
  6. RH-P12-RN 그리퍼 80x80 박스 완충 파지(pos=480, 250mA) 적용
=====================================================================
"""

import os
import sys
import json
import time
import argparse
from pathlib import Path

import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
import DR_init

# =====================================================================
#  ★ [사용자 직접 설정 영역] USER CONFIGURATION
# =====================================================================
# 1. 고정 파지 높이 (Base Z 좌표, mm 단위)
#    ★ 책상 상판: Z = -30.0mm
#    설정값: -6.0mm (사용자 지정 고정 파지 높이)
FIXED_GRASP_Z = -6.0

# 2. 책상 충돌 방지 최저 한계선 (Desk Safety Limit, mm 단위)
DESK_SAFETY_LIMIT_Z = -28.0

# 3. 캘리브레이션 X, Y 미세 보정 오프셋 (mm 단위)
OFFSET_X = -25.5
OFFSET_Y = 0.0

# 4. 상공 안전 대기 높이 (Base Z 좌표, mm 단위)
HOVER_SAFE_Z = 60.0

# 5. 박스 규격 및 그리퍼 닫힘 설정 (80x80x15mm)
BLOCK_HEIGHT_MM = 15.0          # 블록 1단 두께 (mm, 실물 규격: 15mm)
DROP_OFFSET_Z = 10.0            # 적재 릴리즈 여유 높이 (적재 위치보다 +10mm 위에서 오픈)
TARGET_OBJECT_WIDTH_MM = 80.0   # 파지 대상 물체 폭 (mm)
GRIPPER_MAX_STROKE_MM = 106.0   # RH-P12-RN 최대 개구 폭 (mm)
GRIPPER_CLOSE_POS = 480         # 완충 파지 닫힘 위치 (~68mm)
GRIPPER_CLOSE_CURRENT = 250     # 파지 전류 (mA)
GRIPPER_WAIT_OPEN = 1.8         # 그리퍼 완전 열림 물리 대기 시간 (초)
GRIPPER_WAIT_CLOSE = 2.0        # 그리퍼 완전 파지 및 토크 안착 물리 대기 시간 (초)

# 6. 기본 적재(Place) 목표 좌표 (Base 기준, mm 단위 및 Yaw 각도)
#    ★ 로봇팔 베이스 바로 옆 (우측: X=220.0, Y=-210.0 mm)
DEFAULT_PLACE_X = 220.0
DEFAULT_PLACE_Y = -210.0
DEFAULT_PLACE_YAW = 0.0

# 7. 이동 속도 (mm/s, deg/s)
VEL = 20

# 8. 그리퍼 끝단(TCP) 길이 오프셋 (RH-P12-RN: 116.5mm)
TCP_Z = 116.5

# 9. 박스 회전 각도(Yaw) 미세 보정 오프셋 (deg)
YAW_OFFSET = 0.0

# 10. 표준 레시피 사전 정의
RECIPE_PRESETS = {
    "1": {
        "name": "클래식 5단 버거",
        "items": ["Bottom Bun", "Patty", "Cheese", "Tomato", "Bun Top"]
    },
    "2": {
        "name": "치즈버거 4단",
        "items": ["Bottom Bun", "Patty", "Cheese", "Bun Top"]
    },
    "3": {
        "name": "더블 패티 4단",
        "items": ["Bottom Bun", "Patty", "Patty", "Bun Top"]
    },
    "4": {
        "name": "심플 3단 버거",
        "items": ["Bottom Bun", "Patty", "Bun Top"]
    }
}
# =====================================================================

# ── CLI 인자 파싱 ─────────────────────────────────────────────
parser = argparse.ArgumentParser(description="E0509 Burger Autonomous Stacking System")
parser.add_argument("--z", "--grasp-z", type=float, default=None, help="파지 고정 Z 높이 (mm, 기본: -6.0)")
parser.add_argument("--ox", type=float, default=0.0, help="추가 X 오프셋 (mm, 기본: 0.0)")
parser.add_argument("--oy", type=float, default=0.0, help="추가 Y 오프셋 (mm, 기본: 0.0)")
parser.add_argument("--yaw-offset", type=float, default=0.0, help="그리퍼 회전각 보정 오프셋 (deg, 기본: 0.0)")
parser.add_argument("--block-height", type=float, default=15.0, help="블록 1단 높이 (mm, 기본: 15.0)")
parser.add_argument("--drop-offset", type=float, default=10.0, help="적재 릴리즈 여유 높이 (mm, 기본: 10.0)")
parser.add_argument("--place-x", type=float, default=None, help=f"적재 목표 X 좌표 (mm, 기본: {DEFAULT_PLACE_X})")
parser.add_argument("--place-y", type=float, default=None, help=f"적재 목표 Y 좌표 (mm, 기본: {DEFAULT_PLACE_Y})")
parser.add_argument("--place-yaw", type=float, default=0.0, help="적재 목표 회전각 (deg, 기본: 0.0)")
parser.add_argument("--recipe", type=str, default=None, help="적재할 재료 순서 (쉼표 구분, 예: 'Bottom Bun,Patty,Cheese,Bun Top')")
args, _ = parser.parse_known_args()

if args.z is not None:
    FIXED_GRASP_Z = float(args.z)
OFFSET_X += args.ox
OFFSET_Y += args.oy
YAW_OFFSET += args.yaw_offset
BLOCK_HEIGHT_MM = float(args.block_height)
DROP_OFFSET_Z = float(args.drop_offset)

PLACE_X = float(args.place_x) if args.place_x is not None else DEFAULT_PLACE_X
PLACE_Y = float(args.place_y) if args.place_y is not None else DEFAULT_PLACE_Y
PLACE_YAW = float(args.place_yaw)

# ── ROS 2 및 DSR 초기화 ─────────────────────────────────────
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("hamburger_final", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from dsr_msgs2.srv import SetRobotControl
import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, posj, posx, get_current_posx, get_current_posj,  # noqa: E402
                        get_robot_mode, get_robot_state, check_motion, get_last_alarm,
                        DR_BASE, DR_MV_MOD_ABS)
from dsr_gripper import gripper_open, gripper_close, gripper_cmd  # noqa: E402
from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from ultralytics import YOLO  # noqa: E402

_srv_set_robot_control = node.create_client(SetRobotControl, f"/{ROBOT_ID}/dsr_controller2/system/set_robot_control")

TARGETS_JSON = Path("/home/omen/doosan_ws/latest_targets.json")
CONFIG_PATH = Path("~/HamdEyeCal/handeye_config.json").expanduser()
CALIB_PATH = Path("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy").expanduser()
YOLO_MODEL_PATH = Path("/home/omen/doosan_ws/yolo_runs/burger_yolov8n/weights/best.pt")
READY_JOINTS = (0, 0, 90, 0, 90, 0)

if CONFIG_PATH.exists():
    try:
        cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        TCP_Z = float(cfg.get("tcp_z_mm", 116.5))
    except Exception:
        pass

# ── YOLOv8 및 RealSense 카메라 인터페이스 초기화 ───────────────
yolo_model = None
if YOLO_MODEL_PATH.exists():
    try:
        yolo_model = YOLO(str(YOLO_MODEL_PATH))
        print(f"🤖 [YOLOv8n] 최신 모델 로드 완료: {YOLO_MODEL_PATH.name}")
    except Exception as e:
        print(f"⚠️ [YOLOv8n] 모델 로드 실패: {e}")

T_BASE_CAM = None
if CALIB_PATH.exists():
    try:
        T_BASE_CAM = np.load(str(CALIB_PATH))
    except Exception:
        pass

cv_bridge = CvBridge()
latest_color_frame = None
latest_depth_frame = None
cam_intrinsics = {"fx": 386.35, "fy": 385.76, "cx": 321.34, "cy": 244.53}


def _cam_color_cb(msg):
    global latest_color_frame
    try:
        latest_color_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:
        pass


def _cam_depth_cb(msg):
    global latest_depth_frame
    try:
        latest_depth_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception:
        pass


def _cam_info_cb(msg):
    global cam_intrinsics
    cam_intrinsics["fx"] = msg.k[0]
    cam_intrinsics["fy"] = msg.k[4]
    cam_intrinsics["cx"] = msg.k[2]
    cam_intrinsics["cy"] = msg.k[5]


node.create_subscription(Image, "/camera/camera/color/image_raw", _cam_color_cb, 10)
node.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", _cam_depth_cb, 10)
node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", _cam_info_cb, 10)


def extract_box_yaw(crop_img):
    """
    박스 ROI 영상에서 OpenCV minAreaRect로 기울기 각도를 산출하고,
    Hand-Eye 행렬(R_base_cam)을 통해 로봇 Base 기준 Yaw 각도로 변환합니다.
    """
    if crop_img is None or crop_img.size == 0:
        return 0.0
    try:
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 30, 120)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 30:
            return 0.0
        rect = cv2.minAreaRect(c)
        box_pts = cv2.boxPoints(rect)
        e1 = box_pts[1] - box_pts[0]
        e2 = box_pts[2] - box_pts[1]
        edge = e1 if np.linalg.norm(e1) >= np.linalg.norm(e2) else e2
        theta_cam = math.atan2(edge[1], edge[0])

        v_cam = np.array([math.cos(theta_cam), math.sin(theta_cam), 0.0])
        r_mat = T_BASE_CAM[:3, :3] if T_BASE_CAM is not None else np.eye(3)
        v_base = r_mat @ v_cam
        yaw_base = math.degrees(math.atan2(v_base[1], v_base[0]))

        opt_yaw = (yaw_base + 90.0) % 180.0 - 90.0
        return round(float(opt_yaw), 1)
    except Exception:
        return 0.0


def clear_alarm_if_any():
    """로봇 비정상 정지 상태 감지 시 안전 리셋"""
    state = get_robot_state()
    if state in [1, 2]:
        return

    alarm = get_last_alarm()
    param_msg = alarm.param[0] if (alarm and alarm.param and len(alarm.param) > 0) else ""
    print(f"\n⚠️  [로봇 비정상 상태 감지] 현재 State={state}, 마지막 알람={alarm.index if alarm else '없음'}")
    if param_msg:
        print(f"   내용: {param_msg}")

    reset_cmd = None
    if state in [5, 9]:
        reset_cmd = 2
    elif state in [3, 10]:
        reset_cmd = 3
    elif state == 8:
        reset_cmd = 7

    if reset_cmd is not None and _srv_set_robot_control.wait_for_service(timeout_sec=1.0):
        print(f"🔄 로봇 알람 복구 시도 중 (set_robot_control {reset_cmd})...")
        req = SetRobotControl.Request()
        req.robot_control = reset_cmd
        future = _srv_set_robot_control.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.5)


def wait_idle(timeout=30.0):
    """로봇 모션 완료(IDLE) 대기"""
    t0 = time.time()
    time.sleep(0.3)
    while check_motion() != 0:
        if time.time() - t0 > timeout:
            raise RuntimeError("로봇 이동 시간 초과 - 안전을 위해 정지합니다.")
        time.sleep(0.1)
    time.sleep(0.2)
    clear_alarm_if_any()


def get_current_tip():
    """현재 그리퍼 끝단(Fingertip)의 Base 좌표 반환"""
    pose, _ = get_current_posx()
    R = Rot.from_euler("ZYZ", pose[3:6], degrees=True).as_matrix()
    tip_xyz = np.array(pose[:3]) + R @ [0, 0, TCP_Z]
    return tip_xyz, list(pose)


def check_reachability(target_x, target_y, target_z, target_yaw=0.0):
    """사전 역기구학(IK) 및 관절 한계 마진 검증"""
    flange_z = target_z + TCP_Z
    cmd_pose = [float(target_x), float(target_y), float(flange_z), float(target_yaw), 180.0, 0.0]

    for sol in [2, 0]:
        try:
            j = dsr.ikin(posx(*cmd_pose), sol)
            if j is None or isinstance(j, int) or len(j) != 6:
                continue
            if not (-93.0 <= j[1] <= 93.0 and -130.0 <= j[4] <= 130.0 and -155.0 <= j[2] <= 155.0):
                continue
            fk = dsr.fkin(posj(*j))
            if fk is None or isinstance(fk, int) or len(fk) < 3:
                continue
            err = np.linalg.norm(np.array(fk[:3]) - np.array(cmd_pose[:3]))
            if err < 5.0:
                return True, list(j), sol, err
        except Exception:
            continue
    return False, None, None, 999.0


def move_to_hover(target_x, target_y, hover_z, target_yaw=0.0):
    """사전 검증된 관절 각도(movej)로 상공 정렬 이동"""
    clear_alarm_if_any()
    reachable, joints, sol, _ = check_reachability(target_x, target_y, hover_z, target_yaw)
    if not reachable:
        print(f"\n❌ [도달 불가] 목표 상공 위치 (X={target_x:.1f}, Y={target_y:.1f}, Z={hover_z:.1f}mm, Yaw={target_yaw:+.1f}°)는 작업 반경 밖입니다!")
        return False

    movej(posj(*joints), vel=VEL, acc=VEL)
    wait_idle()
    return True


def move_tip_vertical(target_z, target_yaw=None):
    """현재 위치에서 사전 정렬된 Yaw 자세를 유지하며 순수 수직(Z축)으로만 직선(movel) 이동"""
    clear_alarm_if_any()
    target_z = max(DESK_SAFETY_LIMIT_Z, target_z)
    cur_tip, cur_pose = get_current_tip()
    yaw = float(target_yaw) if target_yaw is not None else float(cur_pose[3])
    R = Rot.from_euler("ZYZ", [yaw, 180.0, 0.0], degrees=True).as_matrix()
    target_tip_xyz = np.array([cur_tip[0], cur_tip[1], target_z])
    flange_xyz = target_tip_xyz - R @ [0, 0, TCP_Z]
    cmd_pose = posx(*map(float, flange_xyz), yaw, 180.0, 0.0)
    ret = movel(cmd_pose, vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    if ret != 0:
        alarm = get_last_alarm()
        print(f"❌ 수직 이동 실패: {alarm.param[0] if alarm and alarm.param else '알 수 없는 오류'}")
        clear_alarm_if_any()
        return False
    wait_idle()
    return True


def load_latest_targets(timeout=1.2):
    """RealSense 카메라에서 최신 프레임을 획득하고 YOLOv8 실시간 추론 실행"""
    global latest_color_frame, latest_depth_frame

    latest_color_frame = None
    latest_depth_frame = None

    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if latest_color_frame is not None and latest_depth_frame is not None:
            break

    if yolo_model is not None and latest_color_frame is not None and latest_depth_frame is not None:
        try:
            color = latest_color_frame.copy()
            depth = latest_depth_frame.copy()
            img_h, img_w = color.shape[:2]

            results = yolo_model.predict(color, conf=0.35, device=0, verbose=False)
            targets = {}

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    cls_name = yolo_model.names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    x1 = max(0, min(img_w - 1, x1))
                    x2 = max(0, min(img_w - 1, x2))
                    y1 = max(0, min(img_h - 1, y1))
                    y2 = max(0, min(img_h - 1, y2))
                    if x2 - x1 < 15 or y2 - y1 < 15:
                        continue

                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2

                    crop_box = color[y1:y2, x1:x2]
                    detected_yaw = extract_box_yaw(crop_box)

                    patch = depth[max(0, v - 3):min(img_h, v + 4), max(0, u - 3):min(img_w, u + 4)]
                    valid_d = patch[patch > 0]
                    depth_mm = float(np.median(valid_d)) if len(valid_d) > 0 else (float(depth[v, u]) if depth[v, u] > 0 else 920.0)
                    if depth_mm < 300 or depth_mm > 1500:
                        depth_mm = 920.0

                    xc = (u - cam_intrinsics["cx"]) * depth_mm / cam_intrinsics["fx"]
                    yc = (v - cam_intrinsics["cy"]) * depth_mm / cam_intrinsics["fy"]
                    zc = depth_mm

                    bx, by, bz = 0.0, 0.0, 0.0
                    if T_BASE_CAM is not None:
                        p_cam = np.array([xc / 1000.0, yc / 1000.0, zc / 1000.0, 1.0])
                        p_base = T_BASE_CAM @ p_cam
                        bx = float(p_base[0] * 1000.0)
                        by = float(p_base[1] * 1000.0)
                        bz = float(p_base[2] * 1000.0)

                    if cls_name not in targets or conf > targets[cls_name].get("conf", 0.0):
                        targets[cls_name] = {
                            "conf": conf,
                            "base_x": bx,
                            "base_y": by,
                            "base_z": bz,
                            "yaw_deg": float(detected_yaw),
                            "pixel_uv": [u, v]
                        }

            if targets:
                try:
                    with open(TARGETS_JSON, "w") as f:
                        json.dump(targets, f, indent=2)
                except Exception:
                    pass
                return targets
        except Exception as e:
            print(f"⚠️ [YOLO 직접 추론 오류] {e}")

    if TARGETS_JSON.exists():
        try:
            with open(TARGETS_JSON, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def safe_gripper_open(wait_sec=None):
    if wait_sec is None:
        wait_sec = GRIPPER_WAIT_OPEN
    print(f"🖐️ 그리퍼 열기 중 (완전 전개 대기: {wait_sec:.1f}초)...")
    try:
        ok = gripper_open()
        time.sleep(wait_sec)
        return ok
    except Exception as e:
        print(f"❌ 그리퍼 오류: {e}")
        return False


def safe_gripper_close(pos=None, current=None, wait_sec=None):
    if pos is None:
        pos = GRIPPER_CLOSE_POS
    if current is None:
        current = GRIPPER_CLOSE_CURRENT
    if wait_sec is None:
        wait_sec = GRIPPER_WAIT_CLOSE
    print(f"✊ 그리퍼 파지 중... [목표 pos={pos}, 전류={current}mA, 파지 안착 대기: {wait_sec:.1f}초]")
    try:
        ok = gripper_cmd(pos, current=current)
        time.sleep(wait_sec)
        return ok
    except Exception as e:
        print(f"❌ 그리퍼 오류: {e}")
        return False


def pick_and_place_ingredient(name, target_info, layer_idx, place_x, place_y, place_yaw=0.0):
    """
    단일 재료에 대한 Pick & Place N단 적재 루틴
    - layer_idx: 0 (1층), 1 (2층), 2 (3층), ...
    - place_z: FIXED_GRASP_Z + (layer_idx * BLOCK_HEIGHT_MM)
    """
    # 1. Pick 좌표 계산
    pick_x = target_info["base_x"] + OFFSET_X
    pick_y = target_info["base_y"] + OFFSET_Y
    pick_yaw = (target_info.get("yaw_deg", 0.0) + YAW_OFFSET + 90.0) % 180.0 - 90.0
    pick_z = max(DESK_SAFETY_LIMIT_Z, FIXED_GRASP_Z)

    # 2. Place 좌표 계산 (누적 높이 및 +10mm 릴리즈 여유 반영)
    place_z = FIXED_GRASP_Z + (layer_idx * BLOCK_HEIGHT_MM)
    release_z = place_z + DROP_OFFSET_Z   # 적재 위치보다 10mm 위에서 그리퍼를 열어 부드럽게 안착
    # 이미 쌓여 있는 탑과의 충돌 방지를 위한 접근 상공 높이
    approach_hover_z = max(HOVER_SAFE_Z, release_z + 30.0)

    print(f"\n" + "=" * 65)
    print(f" 🍔 [{layer_idx + 1}단 적재 시작] 재료: '{name}'")
    print(f"   👉 [Pick]  X={pick_x:6.1f}, Y={pick_y:6.1f}, Z={pick_z:5.1f} mm | Yaw={pick_yaw:+5.1f}°")
    print(f"   👉 [Place] X={place_x:6.1f}, Y={place_y:6.1f}, Z={place_z:5.1f} mm (릴리즈 Z={release_z:5.1f} mm, +{DROP_OFFSET_Z:.0f}mm) | Yaw={place_yaw:+5.1f}°")
    print(f"   👉 [Approach Hover] Z={approach_hover_z:5.1f} mm")
    print("=" * 65)

    # 도달성 사전 검증
    ok_pick_hover, _, _, _ = check_reachability(pick_x, pick_y, HOVER_SAFE_Z, pick_yaw)
    ok_pick_down, _, _, _ = check_reachability(pick_x, pick_y, pick_z, pick_yaw)
    ok_place_hover, _, _, _ = check_reachability(place_x, place_y, approach_hover_z, place_yaw)
    ok_place_down, _, _, _ = check_reachability(place_x, place_y, release_z, place_yaw)

    if not (ok_pick_hover and ok_pick_down):
        print(f"❌ [도달 불가] '{name}' Pick 위치가 로봇 작업 반경 밖입니다!")
        return False
    if not (ok_place_hover and ok_place_down):
        print(f"❌ [도달 불가] Place 릴리즈 위치 (Z={release_z:.1f}mm)가 로봇 작업 반경 밖입니다!")
        return False

    # [단계 1] 준비자세 이동 & 그리퍼 열기
    print("  [1/8] 준비자세(READY) 복귀...")
    movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
    wait_idle()
    safe_gripper_open()

    # [단계 2] Pick 상공 진입 (J6 사전 회전 정렬)
    print(f"  [2/8] '{name}' 상공(Z={HOVER_SAFE_Z:.0f}mm) 정렬 이동...")
    if not move_to_hover(pick_x, pick_y, HOVER_SAFE_Z, target_yaw=pick_yaw):
        return False

    # [단계 3] 수직 하강하여 파지 위치 도달
    print(f"  [3/8] 파지 높이(Z={pick_z:.1f}mm)로 수직 하강...")
    if not move_tip_vertical(pick_z, target_yaw=pick_yaw):
        return False

    # [단계 4] 그리퍼 파지 (Grip Close)
    print(f"  [4/8] '{name}' 파지 실행...")
    safe_gripper_close()

    # [단계 5] 물체를 들고 상공으로 수직 상승
    print(f"  [5/8] 안전 상공(Z={approach_hover_z:.0f}mm)으로 수직 상승...")
    if not move_tip_vertical(approach_hover_z, target_yaw=pick_yaw):
        return False

    # [단계 6] 적재(Place) 위치 상공으로 이동 (목표 Yaw로 회전 정렬)
    print(f"  [6/8] 적재 위치 상공 (X={place_x:.1f}, Y={place_y:.1f}, Z={approach_hover_z:.1f}mm) 이동...")
    if not move_to_hover(place_x, place_y, approach_hover_z, target_yaw=place_yaw):
        return False

    # [단계 7] 이번 층 적재 위치보다 10mm 위(release_z)로 수직 하강 후 그리퍼 열기
    print(f"  [7/8] {layer_idx + 1}단 적재 높이보다 10mm 위(Z={release_z:.1f}mm)로 하강...")
    if not move_tip_vertical(release_z, target_yaw=place_yaw):
        return False

    print(f"  [7/8+] 그리퍼 열기 (적재면 +{DROP_OFFSET_Z:.0f}mm 상공 안착 완료)...")
    safe_gripper_open()

    # [단계 8] 적재물 손상 방지를 위해 다시 수직 상승 후 완료
    print(f"  [8/8] 적재 상공(Z={approach_hover_z:.0f}mm)으로 안전 수직 상승...")
    move_tip_vertical(approach_hover_z, target_yaw=place_yaw)

    # 안전하게 준비자세로 복귀
    movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
    wait_idle()

    print(f"🎉 ✅ [{layer_idx + 1}단 적재 완료!] '{name}' 탑 적재 성공 (표면 높이: {place_z:.1f}mm, 릴리즈: {release_z:.1f}mm)")
    return True


def execute_stacking_sequence(recipe_items, place_x, place_y, place_yaw=0.0):
    """지정된 레시피 목록에 따라 순차적으로 N단 적재 수행"""
    total_tiers = len(recipe_items)
    print("\n" + "🍔" * 35)
    print(f"  🚀 총 {total_tiers}단 버거 조립 시퀀스 시작!")
    print(f"     목표 적재 위치: X = {place_x:.1f} mm, Y = {place_y:.1f} mm (Yaw = {place_yaw:+.1f}°)")
    print("     조립 순서:")
    for idx, item in enumerate(recipe_items, 1):
        print(f"       Layer {idx}: {item}")
    print("🍔" * 35)

    time.sleep(1.0)

    for layer_idx, item_name in enumerate(recipe_items):
        print(f"\n🔍 [Layer {layer_idx + 1}/{total_tiers}] '{item_name}' 실시간 비전 탐색 중...")

        # 최신 비전 좌표 갱신 (로봇이 READY 상태에서 카메라 시야를 봄)
        targets = load_latest_targets(timeout=1.5)
        if item_name not in targets:
            print(f"⚠️  '{item_name}' 감지 실패! 1초 후 재시도...")
            time.sleep(1.0)
            targets = load_latest_targets(timeout=2.0)

        if item_name not in targets:
            print(f"❌ [오류] 화면에 '{item_name}' 재료를 찾을 수 없습니다!")
            print(f"   현재 감지된 재료: {list(targets.keys())}")
            ans = input(f"   👉 '{item_name}'을 책상에 올린 후 Enter를 누르거나, [s] 건너뛰기 / [q] 중단: ").strip().lower()
            if ans == "s":
                continue
            elif ans == "q":
                print("🛑 적재 시퀀스를 중단합니다.")
                return False
            # 다시 탐색
            targets = load_latest_targets(timeout=2.0)
            if item_name not in targets:
                print(f"❌ 여전히 '{item_name}'이 감지되지 않아 이 단계를 건너뜁니다.")
                continue

        target_info = targets[item_name]
        success = pick_and_place_ingredient(
            name=item_name,
            target_info=target_info,
            layer_idx=layer_idx,
            place_x=place_x,
            place_y=place_y,
            place_yaw=place_yaw
        )
        if not success:
            print(f"❌ [중단] {layer_idx + 1}단 ({item_name}) 적재 중 오류가 발생했습니다.")
            ans = input("   계속 진행하시겠습니까? (y/N): ").strip().lower()
            if ans != "y":
                return False

    print("\n" + "🎊" * 35)
    print(f"  🏆 축하합니다! 총 {total_tiers}단 수제버거 조립이 성공적으로 완공되었습니다!")
    print(f"  최종 완성 높이: {FIXED_GRASP_Z + ((total_tiers - 1) * BLOCK_HEIGHT_MM):.1f} mm")
    print("🎊" * 35 + "\n")
    return True


def main():
    global FIXED_GRASP_Z, OFFSET_X, OFFSET_Y, YAW_OFFSET, PLACE_X, PLACE_Y, PLACE_YAW, BLOCK_HEIGHT_MM, DROP_OFFSET_Z

    print("\n" + "=" * 70)
    print(" 🍔 Doosan E0509 비전 기반 자율 N단 수제버거 적재 시스템")
    print(f"   ★ 지정 파지 높이 = {FIXED_GRASP_Z:.1f} mm (블록 1단 = {BLOCK_HEIGHT_MM:.0f}mm, 릴리즈 여유 = +{DROP_OFFSET_Z:.0f}mm)")
    print(f"   ★ 적용 오프셋   = X: {OFFSET_X:+.1f} mm, Y: {OFFSET_Y:+.1f} mm, Yaw: {YAW_OFFSET:+.1f}°")
    print(f"   ★ 기본 적재 위치 = X: {PLACE_X:.1f} mm, Y: {PLACE_Y:.1f} mm (Yaw: {PLACE_YAW:+.1f}°)")
    print(f"   ★ 그리퍼 파지   = pos {GRIPPER_CLOSE_POS} (~68mm 완충), 전류: {GRIPPER_CLOSE_CURRENT}mA")
    print("=" * 70)

    # 1. 로봇 연결 확인
    print("\n[1/2] 로봇 드라이버 서비스 연결 확인 중...")
    if not dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=5.0):
        print("❌ [오류] 로봇 서비스 응답 없음! Bringup 실행을 확인하세요.")
        return

    clear_alarm_if_any()
    mode = get_robot_mode()
    state = get_robot_state()
    print(f"✅ 로봇 연결 성공 (모드: {mode}, 상태: {state})")

    # 2. 초기 위치(READY) 이동
    init_move = input("\n👉 로봇을 초기 위치(READY: 카메라 시야 확보)로 이동시키겠습니까? (Enter: 이동 / n: 건너뛰기): ").strip().lower()
    if init_move != 'n':
        print("🚀 로봇을 초기 위치(READY)로 이동 중...")
        movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
        wait_idle()
        print("✅ 초기 위치 도착 완료! 카메라 시야 확보됨 (1.0초 대기)...")
        time.sleep(1.0)

    # 3. CLI에 recipe 인자가 주어진 경우 바로 실행
    if args.recipe:
        recipe_list = [x.strip() for x in args.recipe.split(",") if x.strip()]
        if recipe_list:
            print(f"📋 CLI 지정 레시피 실행: {recipe_list}")
            execute_stacking_sequence(recipe_list, PLACE_X, PLACE_Y, PLACE_YAW)
            return

    # 4. 대화형 메인 루프
    while True:
        targets = load_latest_targets()
        print("\n" + "─" * 70)
        print("🎯 현재 감지된 재료 목록:")
        target_keys = list(targets.keys()) if targets else []
        if target_keys:
            for idx, name in enumerate(target_keys, 1):
                info = targets[name]
                bx = info["base_x"] + OFFSET_X
                by = info["base_y"] + OFFSET_Y
                yw = (info.get("yaw_deg", 0.0) + YAW_OFFSET + 90.0) % 180.0 - 90.0
                conf_str = f"({info['conf']*100:.0f}%)" if "conf" in info else ""
                print(f"  [{idx}] {name:<12}: X={bx:6.1f}, Y={by:6.1f} mm, Yaw={yw:+5.1f}° {conf_str}")
        else:
            print("  ⚠️ 현재 감지된 재료가 없습니다. (카메라 시야 확인)")

        print("─" * 70)
        print("📋 버거 적재 및 제어 메뉴:")
        print("  [1] 레시피 프리셋 선택하여 N단 자율 적재")
        print("  [2] 사용자 직접 재료 순서 지정하여 N단 적재")
        print("  [3] 적재 목표 위치(X, Y, Yaw) 변경")
        print("  [4] 첫 번째 재료 위치를 적재 베이스 위치로 자동 지정")
        print("  [5] 파지 Z / 오프셋 / 블록 두께 설정 변경")
        print("  [h] 준비자세(READY) 복귀")
        print("  [g] 그리퍼 닫기 (pos=480)")
        print("  [o] 그리퍼 열기 (106mm)")
        print("  [r] 실시간 화면 재인식 (YOLO Reload)")
        print("  [q] 종료 (Quit)")
        print(f"  현재 적재 위치: Place X = {PLACE_X:.1f} mm, Y = {PLACE_Y:.1f} mm, Yaw = {PLACE_YAW:+.1f}°")
        print("─" * 70)

        sel = input("👉 메뉴 선택 (1~5 / h / g / o / r / q): ").strip().lower()

        if sel == "q":
            print("프로그램을 종료합니다.")
            break
        elif sel == "r":
            print("🔄 최신 카메라 영상에서 재료 좌표 재인식 중...")
            continue
        elif sel == "h":
            print("🚀 준비자세(READY) 이동...")
            movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
            wait_idle()
            continue
        elif sel == "g":
            safe_gripper_close()
            continue
        elif sel == "o":
            safe_gripper_open()
            continue
        elif sel == "3":
            print(f"\n📍 적재 목표 위치 선택 (현재: X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm, Yaw={PLACE_YAW:.1f}°)")
            print("  [1] 로봇 베이스 우측 바로 옆 (X=220.0, Y=-210.0 mm)")
            print("  [2] 로봇 베이스 좌측 바로 옆 (X=220.0, Y=+210.0 mm)")
            print("  [3] 로봇 베이스 정면 근처     (X=280.0, Y=0.0 mm)")
            print("  [4] 직접 X, Y, Yaw 수치 입력")
            sub_p = input("👉 선택 (1~4 / 취소: c) [1]: ").strip()
            if sub_p == "1" or sub_p == "":
                PLACE_X, PLACE_Y, PLACE_YAW = 220.0, -210.0, 0.0
                print(f"✅ 적재 위치 변경 완료: 로봇 베이스 우측 바로 옆 (X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm)")
            elif sub_p == "2":
                PLACE_X, PLACE_Y, PLACE_YAW = 220.0, 210.0, 0.0
                print(f"✅ 적재 위치 변경 완료: 로봇 베이스 좌측 바로 옆 (X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm)")
            elif sub_p == "3":
                PLACE_X, PLACE_Y, PLACE_YAW = 280.0, 0.0, 0.0
                print(f"✅ 적재 위치 변경 완료: 로봇 베이스 정면 근처 (X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm)")
            elif sub_p == "4":
                try:
                    raw = input("새 적재 좌표 입력 (형식: X Y [Yaw]): ").strip().split()
                    if len(raw) >= 2:
                        PLACE_X = float(raw[0])
                        PLACE_Y = float(raw[1])
                        if len(raw) >= 3:
                            PLACE_YAW = float(raw[2])
                        print(f"✅ 적재 위치 변경 완료: X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm, Yaw={PLACE_YAW:.1f}°")
                    else:
                        print("❌ 공백으로 구분된 두 개 이상의 숫자를 입력하세요.")
                except ValueError:
                    print("❌ 올바른 숫자를 입력하세요.")
            continue
        elif sel == "4":
            if targets and "Bottom Bun" in targets:
                PLACE_X = targets["Bottom Bun"]["base_x"] + OFFSET_X
                PLACE_Y = targets["Bottom Bun"]["base_y"] + OFFSET_Y
                PLACE_YAW = (targets["Bottom Bun"].get("yaw_deg", 0.0) + YAW_OFFSET + 90.0) % 180.0 - 90.0
                print(f"✅ 'Bottom Bun' 위치를 기본 적재 위치로 지정: X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm, Yaw={PLACE_YAW:+.1f}°")
            elif target_keys:
                first_name = target_keys[0]
                PLACE_X = targets[first_name]["base_x"] + OFFSET_X
                PLACE_Y = targets[first_name]["base_y"] + OFFSET_Y
                PLACE_YAW = (targets[first_name].get("yaw_deg", 0.0) + YAW_OFFSET + 90.0) % 180.0 - 90.0
                print(f"✅ '{first_name}' 위치를 기본 적재 위치로 지정: X={PLACE_X:.1f}, Y={PLACE_Y:.1f} mm, Yaw={PLACE_YAW:+.1f}°")
            else:
                print("⚠️ 감지된 재료가 없습니다.")
            continue
        elif sel == "5":
            try:
                print(f"현재: 파지 Z={FIXED_GRASP_Z:.1f}mm, 블록 높이={BLOCK_HEIGHT_MM:.1f}mm, 릴리즈 여유={DROP_OFFSET_Z:.1f}mm, X오프셋={OFFSET_X:.1f}mm")
                val_z = input(f"새 파지 Z (Enter: 유지 {FIXED_GRASP_Z:.1f}): ").strip()
                if val_z:
                    FIXED_GRASP_Z = float(val_z)
                val_bh = input(f"새 블록 1단 높이 (Enter: 유지 {BLOCK_HEIGHT_MM:.1f}): ").strip()
                if val_bh:
                    BLOCK_HEIGHT_MM = float(val_bh)
                val_do = input(f"새 릴리즈 여유 높이 (Enter: 유지 {DROP_OFFSET_Z:.1f}): ").strip()
                if val_do:
                    DROP_OFFSET_Z = float(val_do)
                val_ox = input(f"새 X 오프셋 (Enter: 유지 {OFFSET_X:.1f}): ").strip()
                if val_ox:
                    OFFSET_X = float(val_ox)
                print(f"✅ 설정 갱신: 파지 Z={FIXED_GRASP_Z:.1f}mm, 블록 높이={BLOCK_HEIGHT_MM:.1f}mm, 릴리즈 여유={DROP_OFFSET_Z:.1f}mm, X오프셋={OFFSET_X:.1f}mm")
            except ValueError:
                print("❌ 올바른 숫자를 입력하세요.")
            continue
        elif sel == "1":
            print("\n📋 [프리셋 레시피 선택]:")
            for k, p in RECIPE_PRESETS.items():
                print(f"  [{k}] {p['name']} ({len(p['items'])}단): {' -> '.join(p['items'])}")
            p_sel = input("👉 선택 (1~4 / 취소: c) [1]: ").strip()
            if p_sel.lower() == "c":
                continue
            if not p_sel:
                p_sel = "1"
            if p_sel in RECIPE_PRESETS:
                chosen_recipe = RECIPE_PRESETS[p_sel]["items"]
                print(f"\n🚀 '{RECIPE_PRESETS[p_sel]['name']}' 선택됨: {chosen_recipe}")
                execute_stacking_sequence(chosen_recipe, PLACE_X, PLACE_Y, PLACE_YAW)
            else:
                print("❌ 유효하지 않은 프리셋 번호입니다.")
            continue
        elif sel == "2":
            print("\n✍️ [사용자 직접 순서 지정 N단 적재]")
            print("  사용 가능한 재료 명칭:")
            print("    1: Bottom Bun (하단 번)")
            print("    2: Patty      (패티)")
            print("    3: Cheese     (치즈)")
            print("    4: Tomato     (토마토)")
            print("    5: Bun Top    (상단 번)")
            print("  예시 입력 방법:")
            print("    - 번호로 입력 (예: 1 2 3 4 5 또는 1 2 5)")
            print("    - 이름으로 입력 (예: Bottom Bun, Patty, Bun Top)")
            user_in = input("👉 적재할 순서 입력: ").strip()
            if not user_in:
                continue

            num_map = {
                "1": "Bottom Bun",
                "2": "Patty",
                "3": "Cheese",
                "4": "Tomato",
                "5": "Bun Top"
            }
            tokens = [t.strip() for t in user_in.replace(",", " ").split() if t.strip()]
            custom_recipe = []
            for t in tokens:
                if t in num_map:
                    custom_recipe.append(num_map[t])
                else:
                    # 이름 부분 일치 확인
                    matched = None
                    for valid_name in num_map.values():
                        if t.lower() in valid_name.lower():
                            matched = valid_name
                            break
                    if matched:
                        custom_recipe.append(matched)

            if not custom_recipe:
                print("❌ 유효한 재료 순서를 파싱하지 못했습니다.")
                continue

            print(f"\n🚀 지정된 커스텀 {len(custom_recipe)}단 레시피: {' -> '.join(custom_recipe)}")
            execute_stacking_sequence(custom_recipe, PLACE_X, PLACE_Y, PLACE_YAW)
            continue
        else:
            print("알 수 없는 명령어입니다.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[사용자 중단] 프로그램을 종료합니다.")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
