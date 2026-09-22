#!/usr/bin/env python3
"""E0509 비전 픽앤플레이스 (04강 노트북의 스크립트 버전).

카메라 창에서 물체 클릭 → 핸드-아이 결과 + 오프셋으로 base 좌표 계산
→ 현재 높이 유지한 채 물체 위로 XY 정렬 → 터미널 메뉴로 하강/파지/놓기.

설정(캘리브 경로·TCP 길이·오프셋·스텝·최저높이)은 '클릭 이동 테스트' 앱과 같은
~/HamdEyeCal/handeye_config.json 에서 읽는다. 놓을 위치는 pick_place_hardcoded.py 로
teach 한 poses.json 의 'place' 를 쓴다 (없으면 놓기 메뉴만 비활성).

사전 조건: bringup + gripper_service 실행 중, 로봇 auto 모드.
실행:  python3 vision_pick_place.py            (놓을 곳: poses.json 의 'place')
       python3 vision_pick_place.py --place place2
"""
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as Rot

CONFIG_PATH = Path("~/HamdEyeCal/handeye_config.json").expanduser()
POSES_PATH = Path(__file__).resolve().parent / "poses.json"
READY = (0, 0, 90, 0, 90, 0)
VEL = 20                                   # 저속 고정 (mm/s, deg/s)
REACH_MIN, REACH_MAX = 150.0, 850.0        # 수평 도달 반경(mm)
DEPTH_MIN, DEPTH_MAX = 0.2, 1.5            # 유효 깊이(m)
WIN = "click object: SPACE=confirm, q=quit"
GRIP_WAIT = 1.5                            # 그리퍼 명령 후 다음 이동까지 대기(초)

# ── 설정 ────────────────────────────────────────────────
CFG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
CALIB = CFG.get("calib_path") or str(Path("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy").expanduser())
T_BASE_CAM = np.load(CALIB)
OFFSET = np.array([CFG.get("offset_x_mm", 0.0), CFG.get("offset_y_mm", 0.0), CFG.get("offset_z_mm", 0.0)])
TCP_Z = float(CFG.get("tcp_z_mm", 200.0))
STEP = float(CFG.get("step_mm", 10.0))
Z_MIN = float(CFG.get("z_min_mm", 50.0))
import argparse  # noqa: E402
_ap = argparse.ArgumentParser()
_ap.add_argument("--place", default="place", help="poses.json 의 놓을 자세 이름")
ARGS = _ap.parse_args()
PLACE = json.loads(POSES_PATH.read_text()).get(ARGS.place) if POSES_PATH.exists() else None
print(f"캘리브 {CALIB}\n오프셋 {OFFSET} mm | TCP {TCP_Z:.0f} | 스텝 {STEP:.0f} | 최저 {Z_MIN:.0f}"
      f" | place {'있음' if PLACE else '없음 (teach place 필요)'}")


# ── 카메라 ──────────────────────────────────────────────
def start_camera():
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
        raise SystemExit("카메라 시작 실패 — USB3 직결 확인")
    align = rs.align(rs.stream.color)
    for _ in range(15):
        align.process(pipe.wait_for_frames())
    return pipe, align


def pixel_to_cam(u, v, depth, win=4):
    """(u,v) 주변 깊이 중앙값으로 카메라 좌표 3D 점(m). 유효 깊이 없으면 None."""
    intr = depth.profile.as_video_stream_profile().get_intrinsics()
    pts = []
    for du in range(-win, win + 1):
        for dv in range(-win, win + 1):
            uu, vv = u + du, v + dv
            if 0 <= uu < intr.width and 0 <= vv < intr.height:
                z = depth.get_distance(uu, vv)
                if DEPTH_MIN <= z <= DEPTH_MAX:
                    pts.append(rs.rs2_deproject_pixel_to_point(intr, [float(uu), float(vv)], z))
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
        f = align.process(pipe.wait_for_frames())
        color, depth = np.asanyarray(f.get_color_frame().get_data()).copy(), f.get_depth_frame()
        if state["uv"]:
            p = pixel_to_cam(*state["uv"], depth)
            target = cam_to_base_mm(p) if p is not None else None
            cv2.drawMarker(color, state["uv"], (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
            txt = (f"base x={target[0]:.0f} y={target[1]:.0f} z={target[2]:.0f} mm"
                   if target is not None else "no valid depth - click elsewhere")
            cv2.putText(color, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
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


# ── 로봇 ────────────────────────────────────────────────
import rclpy  # noqa: E402
import DR_init  # noqa: E402

DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "e0509"
rclpy.init()
node = rclpy.create_node("vision_pick_place", namespace="dsr01")
DR_init.__dsr__node = node
import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, movejx, posj, posx, get_current_posx,  # noqa: E402
                        get_robot_mode, check_motion, DR_BASE, DR_MV_MOD_ABS)
from dsr_gripper import gripper_open, gripper_close  # noqa: E402


def wait_idle(timeout=30.0):
    """movej/movel 응답이 도착 전에 올 수 있어서, check_motion()==0(IDLE)이 될 때까지 기다림."""
    t0 = time.time()
    time.sleep(0.2)                                   # 모션이 시작될 틈
    while check_motion() != 0:                        # 0=IDLE, 1=계산 중, 2=동작 중
        if time.time() - t0 > timeout:
            raise RuntimeError("이동이 끝나지 않음 (시간 초과) — 그리퍼 동작을 하지 않고 멈춥니다")
        time.sleep(0.1)
    time.sleep(0.2)                                   # 정착


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
    wait_idle()                                    # 도착 확인 후에만 그리퍼
    gripper_open(); time.sleep(GRIP_WAIT)
    movel(posx(above), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS)


MENU = ("\n[Enter] 한 스텝 내리기  a) 물체 윗면 +20mm 까지  g) 잡기  o) 열기\n"
        "u) 처음 높이로 올리기  p) place 로 옮겨 놓기  r) 준비자세·다음 물체  q) 종료\n> ")

pipe = None
try:
    assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), "로봇 서비스 없음 — bringup/도메인 확인"
    assert get_robot_mode() == 1, "auto 모드가 아닙니다 — pick_place_hardcoded.py mode auto"
    pipe, align = start_camera()
    while True:
        target = pick_target(pipe, align)
        if target is None:
            break
        print(f"\n🎯 목표(base, 오프셋 포함) x={target[0]:.0f} y={target[1]:.0f} z={target[2]:.0f} mm")
        input("⚠️  준비자세 → 물체 위로 이동합니다. 주변 확인 후 Enter (중단 Ctrl+C) ")
        movej(posj(*READY), vel=VEL, acc=VEL)
        wait_idle()
        gripper_open(); time.sleep(GRIP_WAIT)
        tip, pose = tip_now()
        safe_z = tip[2]
        move_tip([target[0], target[1], safe_z], pose[3:6])
        print("물체 위 정렬 완료 — 그리퍼 끝이 물체 바로 위인지 눈으로 확인하세요.")
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
                if PLACE:
                    go_place(safe_z)
                    print("놓기 완료")
                else:
                    print("place 없음 → pick_place_hardcoded.py teach place")
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
