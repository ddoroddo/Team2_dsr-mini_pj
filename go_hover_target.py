#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
  E0509 Real Robot Interactive Hover, Fixed-Z & Offset Tool (v3)
=====================================================================
1. 사전 기구학(IK/FK) 정밀 도달성 검증으로 알람(1206/1207) 원천 방지
2. 안전 관절 이동(movej) 기반으로 특이점(Singularity) 없는 정밀 상공 진입
3. 책상 상판(-30mm) 기준 고정 파지 높이(-25mm) 및 충돌 방지(-28mm) 적용
4. 캘리브레이션 오차 즉시 보정을 위한 X, Y 실시간 오프셋 기능
5. RH-P12-RN 그리퍼 DRL 자동 복구 및 완벽 제어
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
#    ★ 책상 상판(-30.0mm) 2mm 전에 차단하는 절대 하한선
DESK_SAFETY_LIMIT_Z = -28.0

# 3. 캘리브레이션 X, Y 미세 보정 오프셋 (mm 단위)
#    👉 로봇이 목표물 중심에서 벗어날 때 여기에 오차를 입력하세요!
#    OFFSET_X: + 면 로봇 앞쪽(카메라 반대), - 면 로봇 뒤쪽(로봇 베이스 쪽)
#    OFFSET_Y: + 면 로봇 기준 좌측, - 면 로봇 기준 우측
OFFSET_X = -25.5
OFFSET_Y = 0.0

# 4. 상공 안전 대기 높이 (Base Z 좌표, mm 단위)
#    ★ 하향 파지 시 E0509 작업 반경 확보를 위해 60.0mm 적용 (책상 상판 기준 90mm 공중)
HOVER_SAFE_Z = 60.0

# 5. 단계별 미세 하강/상승 스텝 단위 (mm)
STEP_MM = 5.0

# 6. 안전 이동 속도 (mm/s, deg/s)
VEL = 20

# 7. 그리퍼 끝단(TCP) 길이 오프셋 (RH-P12-RN: 116.5mm)
TCP_Z = 116.5

# 8. 파지 대상 물체 폭 및 그리퍼 닫힘 설정 (RH-P12-RN: 최대 스트로크 ~106mm, pos: 0~750)
TARGET_OBJECT_WIDTH_MM = 80.0   # 파지 대상 물체 폭 (mm)
GRIPPER_MAX_STROKE_MM = 106.0   # RH-P12-RN 최대 개구 폭 (mm)
# 80mm 폭 물체에 적절한 파지력을 주기 위한 닫힘 위치 (환산 폭 약 68mm에 해당, 완충 파지)
GRIPPER_CLOSE_POS = 480
GRIPPER_CLOSE_CURRENT = 250     # 파지 전류 (mA, 200~300 권장)

# 9. 박스 회전 각도(Yaw) 미세 보정 오프셋 (deg, 기본: 0.0)
YAW_OFFSET = 0.0
# =====================================================================

# ── CLI 인자 파싱 ─────────────────────────────────────────────
parser = argparse.ArgumentParser(description="E0509 Fixed-Z Grasp Alignment")
parser.add_argument("--z", "--grasp-z", type=float, default=None, help="파지 고정 Z 높이 (mm, 기본: -6.0)")
parser.add_argument("--ox", type=float, default=0.0, help="X 오프셋 (mm)")
parser.add_argument("--oy", type=float, default=0.0, help="Y 오프셋 (mm)")
parser.add_argument("--yaw-offset", type=float, default=0.0, help="그리퍼 회전각(Yaw) 보정 오프셋 (deg, 기본: 0.0)")
parser.add_argument("--width", type=float, default=80.0, help="파지 대상 물체 폭 (mm, 기본: 80.0)")
parser.add_argument("--close-pos", type=int, default=None, help="그리퍼 닫힘 위치 (0~750, 기본: 480)")
parser.add_argument("--current", type=int, default=250, help="그리퍼 파지 전류 (mA, 기본: 250)")
args, _ = parser.parse_known_args()
if args.z is not None:
    FIXED_GRASP_Z = float(args.z)
OFFSET_X += args.ox
OFFSET_Y += args.oy
YAW_OFFSET += args.yaw_offset
TARGET_OBJECT_WIDTH_MM = float(args.width)
GRIPPER_CLOSE_CURRENT = int(args.current)
if args.close_pos is not None:
    GRIPPER_CLOSE_POS = int(args.close_pos)
else:
    # 폭 기준 자동 계산 (대상 폭보다 약 12mm 더 닫는 위치로 완충 파지)
    GRIPPER_CLOSE_POS = int(750 * max(0.0, TARGET_OBJECT_WIDTH_MM - 12.0) / GRIPPER_MAX_STROKE_MM)
    GRIPPER_CLOSE_POS = max(0, min(750, GRIPPER_CLOSE_POS))

# 책상 충돌 방지 검증
if FIXED_GRASP_Z < DESK_SAFETY_LIMIT_Z:
    print(f"⚠️ [책상 보호] 설정된 파지 높이({FIXED_GRASP_Z:.1f}mm)가 책상 한계선({DESK_SAFETY_LIMIT_Z:.1f}mm)보다 낮습니다.")
    FIXED_GRASP_Z = DESK_SAFETY_LIMIT_Z

# ── ROS 2 및 DSR 초기화 ─────────────────────────────────────
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("go_hover_target", namespace=ROBOT_ID)
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
        print(f"🤖 [YOLOv8n] 모델 로드 완료: {YOLO_MODEL_PATH.name} (클래스: {list(yolo_model.names.values())})")
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
    [Phase 1 & Phase 2] 박스 ROI 영상에서 OpenCV minAreaRect로 기울기 각도를 산출하고,
    Hand-Eye 행렬(R_base_cam)을 통해 로봇 Base 기준 Yaw 각도로 변환합니다.
    평행 그리퍼의 180도 대칭성을 고려하여 [-90.0, +90.0] 범위의 최단 회전각을 반환합니다.
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

        # 카메라 2D 벡터 -> 로봇 Base 3D 벡터 투영
        v_cam = np.array([math.cos(theta_cam), math.sin(theta_cam), 0.0])
        r_mat = T_BASE_CAM[:3, :3] if T_BASE_CAM is not None else np.eye(3)
        v_base = r_mat @ v_cam
        yaw_base = math.degrees(math.atan2(v_base[1], v_base[0]))

        # 180도 대칭성 정규화 (케이블 꼬임 및 조인트 리밋 방지: [-90, +90])
        opt_yaw = (yaw_base + 90.0) % 180.0 - 90.0
        return round(float(opt_yaw), 1)
    except Exception:
        return 0.0


def clear_alarm_if_any():
    """
    로봇이 실제 비정상 정지(SAFE_STOP, SAFE_OFF 등) 상태일 때만 안전하게 리셋.
    STATE_STANDBY(1) 또는 STATE_MOVING(2) 정상 상태일 때는 불필요한 제어 명령을 호출하지 않습니다.
    """
    state = get_robot_state()
    # 1: STANDBY, 2: MOVING - 정상 동작 상태이므로 불필요한 리셋 방지
    if state in [1, 2]:
        return

    # 비정상 상태(3:SAFE_OFF, 5:SAFE_STOP, 6:EMERGENCY, 8:RECOVERY 등) 감지 시에만 처리
    alarm = get_last_alarm()
    param_msg = alarm.param[0] if (alarm and alarm.param and len(alarm.param) > 0) else ""
    print(f"\n⚠️  [로봇 비정상 상태 감지] 현재 State={state}, 마지막 알람={alarm.index if alarm else '없음'}")
    if param_msg:
        print(f"   내용: {param_msg}")

    # 두산 공식 리셋 코드 매핑
    reset_cmd = None
    if state in [5, 9]:      # STATE_SAFE_STOP, STATE_SAFE_STOP2
        reset_cmd = 2        # CONTROL_RESET_SAFET_STOP
    elif state in [3, 10]:   # STATE_SAFE_OFF, STATE_SAFE_OFF2
        reset_cmd = 3        # CONTROL_RESET_SAFET_OFF
    elif state == 8:         # STATE_RECOVERY
        reset_cmd = 7        # CONTROL_RESET_RECOVERY

    if reset_cmd is not None and _srv_set_robot_control.wait_for_service(timeout_sec=1.0):
        print(f"🔄 로봇 알람 복구 시도 중 (set_robot_control {reset_cmd})...")
        req = SetRobotControl.Request()
        req.robot_control = reset_cmd
        future = _srv_set_robot_control.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(0.5)


def wait_idle(timeout=30.0):
    """로봇 모션 완료(IDLE) 대기 및 에러 감지"""
    t0 = time.time()
    time.sleep(0.3)
    while check_motion() != 0:
        if time.time() - t0 > timeout:
            raise RuntimeError("로봇 이동 시간 초과 - 안전을 위해 정지합니다.")
        time.sleep(0.1)
    time.sleep(0.2)
    clear_alarm_if_any()


def get_current_tip():
    """현재 그리퍼 끝단(Fingertip)의 Base 좌표 및 플랜지 포즈 반환"""
    pose, _ = get_current_posx()
    R = Rot.from_euler("ZYZ", pose[3:6], degrees=True).as_matrix()
    tip_xyz = np.array(pose[:3]) + R @ [0, 0, TCP_Z]
    return tip_xyz, list(pose)


def check_reachability(target_x, target_y, target_z, target_yaw=0.0):
    """
    사전 역기구학(IK) 및 순기구학(FK) 정밀 검증 함수
    - 목표 위치로 이동하기 전, 실제로 로봇이 도달 가능한지 수학적으로 먼저 확인합니다.
    - target_yaw 각도를 포함하여 J6 관절 각도를 사전에 검증합니다.
    """
    flange_z = target_z + TCP_Z
    cmd_pose = [float(target_x), float(target_y), float(flange_z), float(target_yaw), 180.0, 0.0]

    # sol=2 (READY 자세와 일치하는 팔꿈치 형태) 우선, 이후 sol=0 시도
    for sol in [2, 0]:
        try:
            j = dsr.ikin(posx(*cmd_pose), sol)
            if j is None or isinstance(j, int) or len(j) != 6:
                continue
            # 관절 한계 마진 검증 (J2: [-93, 93], J5: [-130, 130], J3: [-155, 155])
            if not (-93.0 <= j[1] <= 93.0 and -130.0 <= j[4] <= 130.0 and -155.0 <= j[2] <= 155.0):
                continue
            # FK 검증 (IK 해의 실제 플랜지 위치가 명령 위치와 오차 5mm 이내인지)
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
    """
    사전 검증된 관절 각도(movej)를 사용하여 특이점 및 도달불가 에러 없이 안전 상공 진입
    상공에서 J6 관절을 목표 Yaw 각도로 사전 회전(Pre-rotate)하여 정렬합니다.
    """
    clear_alarm_if_any()
    reachable, joints, sol, err = check_reachability(target_x, target_y, hover_z, target_yaw)
    if not reachable:
        print(f"\n❌ [도달 불가] 목표 상공 위치 (X={target_x:.1f}, Y={target_y:.1f}, Z={hover_z:.1f}mm, Yaw={target_yaw:+.1f}°)는 작업 반경 밖입니다!")
        print("   👉 [f] 메뉴에서 OFFSET_X 를 -100 ~ -140 mm 로 조정하거나 작업 반경 안의 재료(Cheese 등)를 선택하세요.")
        return False

    print(f"🚀 사전 IK 검증 통과 (sol={sol}, J6 목표각={joints[5]:+.1f}°) -> 상공 회전 정렬 이동 시작...")
    movej(posj(*joints), vel=VEL, acc=VEL)
    wait_idle()

    # 실제 도달 확인
    cur_tip, _ = get_current_tip()
    pos_err = np.linalg.norm(cur_tip[:2] - np.array([target_x, target_y]))
    if pos_err > 20.0:
        print(f"⚠️ [도달 확인 실패] 실제 위치 오차 {pos_err:.1f}mm")
        return False
    return True


def move_tip_vertical(target_z, target_yaw=None):
    """동일한 X, Y 위치에서 사전 정렬된 Yaw 자세를 유지하며 순수 수직(Z축)으로만 직선(movel) 하강/상승"""
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


def load_latest_targets(timeout=1.0):
    """
    1. RealSense 카메라 영상에서 YOLOv8 모델 직접 실시간 추론 실행
    2. 만약 카메라 프레임이 지연되면 latest_targets.json 파일에서 폴백 로드
    """
    global latest_color_frame, latest_depth_frame

    # 최신 프레임을 실시간으로 새로 수신하기 위해 리셋
    latest_color_frame = None
    latest_depth_frame = None

    # 최신 카메라 프레임 수신 대기 (spin)
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if latest_color_frame is not None and latest_depth_frame is not None:
            break

    # YOLOv8 직접 실시간 추론
    if yolo_model is not None and latest_color_frame is not None and latest_depth_frame is not None:
        try:
            color = latest_color_frame.copy()
            depth = latest_depth_frame.copy()
            img_h, img_w = color.shape[:2]

            results = yolo_model.predict(color, conf=0.40, device=0, verbose=False)
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

                    # 중심점 계산
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2

                    # [Phase 1 & Phase 2] 박스 회전 각도(Yaw) 정밀 추출
                    crop_box = color[y1:y2, x1:x2]
                    detected_yaw = extract_box_yaw(crop_box)

                    # 뎁스 샘플링 (5x5 median patch)
                    patch = depth[max(0, v - 3):min(img_h, v + 4), max(0, u - 3):min(img_w, u + 4)]
                    valid_d = patch[patch > 0]
                    depth_mm = float(np.median(valid_d)) if len(valid_d) > 0 else (float(depth[v, u]) if depth[v, u] > 0 else 920.0)
                    if depth_mm < 300 or depth_mm > 1500:
                        depth_mm = 920.0

                    # 카메라 3D
                    xc = (u - cam_intrinsics["cx"]) * depth_mm / cam_intrinsics["fx"]
                    yc = (v - cam_intrinsics["cy"]) * depth_mm / cam_intrinsics["fy"]
                    zc = depth_mm

                    # 로봇 Base 3D
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
                            "cam_x": xc,
                            "cam_y": yc,
                            "depth_mm": int(depth_mm),
                            "pixel_uv": [u, v],
                            "detector": "YOLOv8n"
                        }

            if targets:
                try:
                    export_dict = {}
                    for k, v in targets.items():
                        d = dict(v)
                        if "conf" in d:
                            del d["conf"]
                        export_dict[k] = d
                    with open(TARGETS_JSON, "w") as f:
                        json.dump(export_dict, f, indent=2)
                except Exception:
                    pass
                return targets
        except Exception as e:
            print(f"⚠️ [YOLO 직접 추론 오류] {e}")

    # 2순위: latest_targets.json 로드
    if TARGETS_JSON.exists():
        try:
            with open(TARGETS_JSON, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def safe_gripper_open():
    print("🖐️ 그리퍼 열기 중 (106mm 전개)...")
    try:
        ok = gripper_open()
        time.sleep(1.0)
        if ok:
            print("✅ 그리퍼 열림 완료")
        else:
            print("⚠️ 그리퍼 열기 응답 실패 (하드웨어 연결 점검 필요)")
        return ok
    except Exception as e:
        print(f"❌ 그리퍼 오류: {e}")
        return False


def safe_gripper_close(pos=None, current=None):
    if pos is None:
        pos = GRIPPER_CLOSE_POS
    if current is None:
        current = GRIPPER_CLOSE_CURRENT
    approx_w = (pos / 750.0) * GRIPPER_MAX_STROKE_MM
    print(f"✊ 그리퍼 파지 중... [목표 위치={pos} (환산 폭 ~{approx_w:.1f}mm), 전류={current}mA]")
    try:
        ok = gripper_cmd(pos, current=current)
        time.sleep(1.0)
        if ok:
            print(f"✅ 그리퍼 파지 완료 (위치: {pos}, 물체 폭: ~{TARGET_OBJECT_WIDTH_MM:.0f}mm 맞춤)")
        else:
            print("⚠️ 그리퍼 닫기 응답 실패 (하드웨어 연결 점검 필요)")
        return ok
    except Exception as e:
        print(f"❌ 그리퍼 오류: {e}")
        return False


def main():
    global FIXED_GRASP_Z, OFFSET_X, OFFSET_Y, YAW_OFFSET, HOVER_SAFE_Z, TARGET_OBJECT_WIDTH_MM, GRIPPER_CLOSE_POS, GRIPPER_CLOSE_CURRENT

    print("\n" + "=" * 70)
    print(" 🤖 Doosan E0509 비전 타겟 고정-Z 파지 정렬 도구 (v3 - J6 회전 파지형)")
    print(f"   ★ 지정 파지 높이 = {FIXED_GRASP_Z:.1f} mm (책상 = -30.0mm)")
    print(f"   ★ 파지 대상 폭   = {TARGET_OBJECT_WIDTH_MM:.1f} mm -> 닫힘 pos={GRIPPER_CLOSE_POS} (전류: {GRIPPER_CLOSE_CURRENT}mA)")
    print(f"   ★ 책상 충돌 한계 = {DESK_SAFETY_LIMIT_Z:.1f} mm (충돌 방지)")
    print(f"   ★ 적용 오프셋   = X: {OFFSET_X:+.1f} mm, Y: {OFFSET_Y:+.1f} mm, Yaw: {YAW_OFFSET:+.1f}°")
    print(f"   ★ 상공 호버 높이 = {HOVER_SAFE_Z:.1f} mm | 속도 = {VEL}mm/s")
    print("=" * 70)

    # 1. 서비스 연결 및 로봇 모드 확인
    print("\n[1/3] 로봇 드라이버 서비스 연결 확인 중...")
    if not dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=5.0):
        print("❌ [오류] 로봇 서비스 응답 없음! Bringup 실행을 확인하세요.")
        return

    clear_alarm_if_any()
    mode = get_robot_mode()
    state = get_robot_state()
    print(f"✅ 로봇 연결 성공 (현재 모드: {mode}, 상태: {state})")
    if mode != 1:
        print("⚠️  [주의] 로봇이 AUTO(1) 모드가 아닙니다. 티칭 펜던트에서 AUTO 모드를 확인하세요.")

    # 2. 실행 시 카메라 시야 확보를 위한 초기 위치(READY) 복귀 확인
    init_move = input("\n👉 로봇을 초기 위치(READY: 카메라 시야 확보)로 이동시키겠습니까? (Enter: 이동 / n: 건너뛰기): ").strip().lower()
    if init_move != 'n':
        print("🚀 로봇을 초기 위치(READY = 0, 0, 90, 0, 90, 0)로 이동 중...")
        movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
        wait_idle()
        print("✅ 초기 위치 도착 완료! 카메라 시야 확보되었습니다. 인식 대기 중 (1.5초)...")
        time.sleep(1.5)

    while True:
        # 3. 실시간 타겟 로드
        targets = load_latest_targets()
        if not targets:
            print("\n⚠️  감지된 재료가 없습니다. (로봇이 카메라를 가리고 있거나 detector 노드 확인)")
            ans = input("   [h] 로봇 초기 위치(READY)로 이동하여 카메라 시야 확보\n   [r] 다시 로드\n   [q] 종료\n👉 선택: ").strip().lower()
            if ans == "h":
                movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
                wait_idle()
                time.sleep(1.5)
                continue
            elif ans == "q":
                break
            time.sleep(0.5)
            continue

        print("\n" + "─" * 70)
        print(f"🎯 현재 감지된 재료 목록 (오프셋 X:{OFFSET_X:+.1f}, Y:{OFFSET_Y:+.1f} mm, Yaw:{YAW_OFFSET:+.1f}° | 파지 Z: {FIXED_GRASP_Z:.1f} mm | 닫힘 pos: {GRIPPER_CLOSE_POS}):")
        target_keys = list(targets.keys())
        reach_status = {}
        target_yaws = {}
        for idx, name in enumerate(target_keys, 1):
            info = targets[name]
            bx = info["base_x"] + OFFSET_X
            by = info["base_y"] + OFFSET_Y
            raw_yaw = info.get("yaw_deg", 0.0)
            opt_yaw = (raw_yaw + YAW_OFFSET + 90.0) % 180.0 - 90.0
            target_yaws[name] = opt_yaw
            ok_hover, joints, _, _ = check_reachability(bx, by, HOVER_SAFE_Z, opt_yaw)
            reach_status[name] = ok_hover
            j6_str = f"J6:{joints[5]:+5.1f}°" if ok_hover else "한계초과"
            stat_str = f"✅ 도달 가능 ({j6_str})" if ok_hover else "❌ 반경 초과 (X오프셋 조정 필요)"
            conf_str = f" (YOLO {info['conf']*100:.0f}%)" if "conf" in info else ""
            print(f"  [{idx}] {name:<12}: Base X={bx:6.1f}, Y={by:6.1f} mm, Yaw={opt_yaw:+5.1f}° {conf_str:<12} [{stat_str}]")

        print("  [h] 로봇 초기 위치(READY)로 이동 (카메라 시야 확보)")
        print(f"  [f] X, Y 오프셋 미세 보정 (현재: X={OFFSET_X:+.1f}, Y={OFFSET_Y:+.1f} mm)")
        print(f"  [y] 그리퍼 회전 각도(Yaw) 오프셋 보정 (현재: {YAW_OFFSET:+.1f}°)")
        print(f"  [z] 파지 고정 Z 높이 변경 (현재: {FIXED_GRASP_Z:.1f} mm)")
        print(f"  [c] 그리퍼 닫힘/파지폭 설정 (현재: pos={GRIPPER_CLOSE_POS}, 대상폭: {TARGET_OBJECT_WIDTH_MM:.0f}mm)")
        print("  [g] 그리퍼 닫기 (Close)")
        print("  [o] 그리퍼 열기 (Open)")
        print("  [r] 실시간 좌표 새로고침 (YOLO Reload)")
        print("  [q] 종료 (Quit)")
        print("─" * 70)

        sel = input("👉 번호(1~5) 또는 메뉴(h/f/y/z/c/g/o/r/q) 입력: ").strip()
        if sel.lower() == "q":
            break
        if sel.lower() == "r":
            print("🔄 [YOLOv8] 최신 카메라 영상에서 재료 좌표 재인식 중...")
            continue
        if sel.lower() == "g":
            safe_gripper_close()
            continue
        if sel.lower() == "o":
            safe_gripper_open()
            continue
        if sel.lower() == "h":
            print("🚀 초기 위치(READY)로 이동 중...")
            movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
            wait_idle()
            print("✅ 초기 위치 도착 완료 (카메라 시야 확보)")
            time.sleep(1.0)
            continue
        if sel.lower() == "y":
            try:
                new_yaw_off = float(input(f"새로운 Yaw 각도 보정값 입력 (현재: {YAW_OFFSET:+.1f}°, 예: +10 또는 -15): ").strip())
                YAW_OFFSET = new_yaw_off
                print(f"✅ Yaw 각도 보정값 변경 완료: {YAW_OFFSET:+.1f}°")
            except ValueError:
                print("❌ 유효한 숫자를 입력해 주세요.")
            continue
        if sel.lower() == "c":
            try:
                print(f"\n현재 설정: 대상 폭 {TARGET_OBJECT_WIDTH_MM:.1f}mm | 닫힘 위치 pos={GRIPPER_CLOSE_POS} (0:완전닫힘 ~ 750:완전열림)")
                val = input("새로운 대상 폭(mm 단위, 예: 80) 또는 직접 pos(예: p480) 입력: ").strip()
                if val.lower().startswith("p"):
                    GRIPPER_CLOSE_POS = max(0, min(750, int(val[1:])))
                    TARGET_OBJECT_WIDTH_MM = (GRIPPER_CLOSE_POS / 750.0) * GRIPPER_MAX_STROKE_MM
                else:
                    TARGET_OBJECT_WIDTH_MM = float(val)
                    GRIPPER_CLOSE_POS = int(750 * max(0.0, TARGET_OBJECT_WIDTH_MM - 12.0) / GRIPPER_MAX_STROKE_MM)
                    GRIPPER_CLOSE_POS = max(0, min(750, GRIPPER_CLOSE_POS))
                approx_w = (GRIPPER_CLOSE_POS / 750.0) * GRIPPER_MAX_STROKE_MM
                print(f"✅ 파지 설정 변경 완료: 대상 폭 {TARGET_OBJECT_WIDTH_MM:.1f}mm -> 목표 닫힘 pos {GRIPPER_CLOSE_POS} (환산 폭 ~{approx_w:.1f}mm)")
            except ValueError:
                print("❌ 유효한 값을 입력해 주세요.")
            continue
        if sel.lower() == "f":
            try:
                raw_off = input(f"새로운 오프셋 입력 (형식: dx dy, 현재 {OFFSET_X:.1f} {OFFSET_Y:.1f}): ").strip().split()
                if len(raw_off) >= 2:
                    OFFSET_X = float(raw_off[0])
                    OFFSET_Y = float(raw_off[1])
                    print(f"✅ 오프셋 변경 완료: X={OFFSET_X:+.1f}mm, Y={OFFSET_Y:+.1f}mm")
                else:
                    print("❌ 공백으로 구분된 두 숫자(dx dy)를 입력해 주세요. (예: -120 0)")
            except ValueError:
                print("❌ 유효한 숫자를 입력해 주세요.")
            continue
        if sel.lower() == "z":
            try:
                new_z = float(input(f"새로운 파지 Z 높이 입력 (현재 {FIXED_GRASP_Z:.1f}mm, 한계 {DESK_SAFETY_LIMIT_Z:.0f}mm): ").strip())
                if new_z < DESK_SAFETY_LIMIT_Z:
                    print(f"❌ [책상 보호] 책상 한계({DESK_SAFETY_LIMIT_Z:.0f}mm) 이상이어야 합니다.")
                else:
                    FIXED_GRASP_Z = new_z
                    print(f"✅ 파지 높이가 {FIXED_GRASP_Z:.1f} mm로 변경되었습니다.")
            except ValueError:
                print("❌ 유효한 숫자를 입력해 주세요.")
            continue

        try:
            choice_idx = int(sel) - 1
            if not (0 <= choice_idx < len(target_keys)):
                print("❌ 유효하지 않은 번호입니다.")
                continue
        except ValueError:
            print("❌ 숫자를 입력해 주세요.")
            continue

        chosen_name = target_keys[choice_idx]
        chosen_info = targets[chosen_name]
        t_x = chosen_info["base_x"] + OFFSET_X
        t_y = chosen_info["base_y"] + OFFSET_Y
        t_z = max(DESK_SAFETY_LIMIT_Z, FIXED_GRASP_Z)
        t_yaw = target_yaws.get(chosen_name, 0.0)

        print(f"\n선택된 목표: [{chosen_name}]")
        print(f"  - 목표 좌표: Base X = {t_x:.1f} mm, Y = {t_y:.1f} mm")
        print(f"  - 파지 각도: Base Yaw = {t_yaw:+.1f}° (그리퍼 J6 회전 정렬)")
        print(f"  - 파지 높이: Base Z = {t_z:.1f} mm (책상 상판: -30.0mm)")
        print(f"  - 상공 호버: Base Z = {HOVER_SAFE_Z:.1f} mm")

        if not reach_status.get(chosen_name, False):
            print(f"\n⚠️  경고: [{chosen_name}]의 현재 좌표는 E0509의 작업 반경(최대 약 720mm)을 초과합니다!")
            print(f"   현재 계산된 X = {t_x:.1f}mm -> [f] 키를 눌러 OFFSET_X 에 -100 ~ -150 mm 를 적용해 보세요.")
            ans = input("   그래도 이동을 시도하시겠습니까? (y/N): ").strip().lower()
            if ans != 'y':
                continue

        mode_sel = input(
            "\n이동 방식 선택:\n"
            f"  [1] 상공 안전 호버(Z={HOVER_SAFE_Z:.0f}mm)로 먼저 정렬 (권장)\n"
            f"  [2] 고정 파지 높이(Z={t_z:.0f}mm)로 직접 정렬 하강\n"
            "👉 선택 (1 또는 2 / 취소: c) [1]: "
        ).strip()

        if mode_sel.lower() == "c":
            print("취소되었습니다.")
            continue

        direct_to_grasp = (mode_sel == "2")

        # 3. 준비자세(READY) 이동
        print("\n1) 준비자세(READY) 경유 중...")
        movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
        wait_idle()

        # 4. 그리퍼 열기
        safe_gripper_open()

        # 5. 목표물 상공으로 안전 이동 (사전 IK 검증 적용 및 J6 사전 회전)
        print(f"2) [{chosen_name}] 상공 Z={HOVER_SAFE_Z:.0f}mm (J6 회전각={t_yaw:+.1f}°) 로 정렬 이동 중...")
        success = move_to_hover(t_x, t_y, HOVER_SAFE_Z, target_yaw=t_yaw)
        if not success:
            print("⚠️ 상공 이동이 중단되었습니다. 오프셋을 조정하거나 다른 재료를 선택하세요.")
            continue

        print(f"✅ [상공 정렬 성공] 그리퍼 끝이 [{chosen_name}] 상공에 도달했습니다. (J6 회전각={t_yaw:+.1f}° 정렬 완료)")

        if direct_to_grasp:
            print(f"3) 고정 파지 높이 Z={t_z:.1f}mm 로 수직 하강 중...")
            move_tip_vertical(t_z, target_yaw=t_yaw)
            print(f"✅ [정렬 완료] 엔드이펙터가 [{chosen_name}] 파지 위치(Z={t_z:.1f}mm)에 도달했습니다!")

        # 6. 대화형 단계별 제어 메뉴
        while True:
            cur_tip, _ = get_current_tip()
            MENU = (
                f"\n[현재 그리퍼 끝단: Z = {cur_tip[2]:.1f} mm | 파지 목표 Z = {t_z:.1f} mm | 파지 Yaw = {t_yaw:+.1f}°]\n"
                f"  [d]     고정 파지 높이(Z={t_z:.1f}mm)로 직접 하강\n"
                f"  [Enter] 아래로 {STEP_MM:.0f}mm 미세 하강\n"
                f"  [y]     그리퍼 파지 회전각(Yaw) 미세 조정 (현재: {t_yaw:+.1f}°)\n"
                f"  [g]     그리퍼 잡기 (Grip Close: pos={GRIPPER_CLOSE_POS})\n"
                f"  [o]     그리퍼 열기 (Grip Open: 106mm)\n"
                f"  [c]     그리퍼 파지 닫힘 설정 (현재 pos={GRIPPER_CLOSE_POS}, 대상 폭={TARGET_OBJECT_WIDTH_MM:.0f}mm)\n"
                f"  [u]     상공 안전 높이(Z={HOVER_SAFE_Z:.0f}mm)로 상승\n"
                "  [r]     재료 선택 메뉴로 복귀 (READY 이동)\n"
                "  [q]     종료\n"
                "> "
            )
            cmd = input(MENU).strip().lower()

            if cmd == "d":  # 고정 파지 높이로 직접 수직 하강
                move_tip_vertical(t_z, target_yaw=t_yaw)
                print(f"👉 고정 파지 높이 도달 완료: Z = {t_z:.1f} mm (Yaw = {t_yaw:+.1f}°)")

            elif cmd == "":  # 한 스텝 미세 하강
                cur_tip, _ = get_current_tip()
                next_z = cur_tip[2] - STEP_MM
                if next_z < DESK_SAFETY_LIMIT_Z:
                    print(f"❌ [책상 보호 거부] 책상 충돌 방지 최저선({DESK_SAFETY_LIMIT_Z:.0f}mm) 이하로는 내려갈 수 없습니다!")
                    continue
                move_tip_vertical(next_z, target_yaw=t_yaw)
                print(f"👉 현재 그리퍼 끝 높이: Z = {next_z:.1f} mm (Yaw = {t_yaw:+.1f}°)")

            elif cmd == "y":
                try:
                    val = input(f"새로운 파지 Yaw 회전각 입력 (현재 {t_yaw:+.1f}°, 예: +30 또는 -15): ").strip()
                    new_yaw = float(val)
                    t_yaw = (new_yaw + 90.0) % 180.0 - 90.0
                    cur_tip, _ = get_current_tip()
                    print(f"🔄 J6 관절을 Yaw={t_yaw:+.1f}° 로 회전 정렬 중...")
                    move_tip_vertical(cur_tip[2], target_yaw=t_yaw)
                    print(f"✅ 그리퍼 각도가 Yaw={t_yaw:+.1f}° 로 회전 정렬되었습니다.")
                except ValueError:
                    print("❌ 올바른 숫자를 입력해 주세요.")

            elif cmd == "g":
                safe_gripper_close()

            elif cmd == "o":
                safe_gripper_open()

            elif cmd == "c":
                try:
                    val = input(f"새로운 대상 폭(mm 단위, 현재 {TARGET_OBJECT_WIDTH_MM:.1f}mm) 또는 pos(p480) 입력: ").strip()
                    if val.lower().startswith("p"):
                        GRIPPER_CLOSE_POS = max(0, min(750, int(val[1:])))
                        TARGET_OBJECT_WIDTH_MM = (GRIPPER_CLOSE_POS / 750.0) * GRIPPER_MAX_STROKE_MM
                    else:
                        TARGET_OBJECT_WIDTH_MM = float(val)
                        GRIPPER_CLOSE_POS = int(750 * max(0.0, TARGET_OBJECT_WIDTH_MM - 12.0) / GRIPPER_MAX_STROKE_MM)
                        GRIPPER_CLOSE_POS = max(0, min(750, GRIPPER_CLOSE_POS))
                    approx_w = (GRIPPER_CLOSE_POS / 750.0) * GRIPPER_MAX_STROKE_MM
                    print(f"✅ 파지 설정 변경 완료: 대상 폭 {TARGET_OBJECT_WIDTH_MM:.1f}mm -> 목표 닫힘 pos {GRIPPER_CLOSE_POS} (환산 폭 ~{approx_w:.1f}mm)")
                except ValueError:
                    print("❌ 올바른 값을 입력해 주세요.")

            elif cmd == "u":
                print(f"⬆️ 상공 안전 높이(Z={HOVER_SAFE_Z:.0f}mm)로 상승...")
                move_tip_vertical(HOVER_SAFE_Z, target_yaw=t_yaw)

            elif cmd == "r":
                print("상공 상승 후 준비자세로 복귀합니다...")
                move_tip_vertical(HOVER_SAFE_Z, target_yaw=t_yaw)
                movej(posj(*READY_JOINTS), vel=VEL, acc=VEL)
                wait_idle()
                break

            elif cmd == "q":
                print("안전을 위해 상공으로 상승 후 종료합니다...")
                move_tip_vertical(HOVER_SAFE_Z, target_yaw=t_yaw)
                return

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
