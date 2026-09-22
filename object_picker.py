#!/usr/bin/env python3
"""요청한 물체 집기 — /detections 토픽(detector_node.py 또는 팀원 검출기)을 받아서 픽앤플레이스.

흐름: 준비자세(카메라를 가리지 않는 자세)에서 최신 검출 대기 → 라벨 입력 → 가장 신뢰도 높은 물체 선택
     → 카메라 3D → base 좌표(핸드-아이 결과 + 오프셋) → 물체 위 → 하강 → 닫기 → 상승
     → place(poses.json) 에 놓기 → 준비자세 → 다시 검출해서 원래 자리에 남아 있으면 경고.

사전 조건: bringup + gripper_service + detector_node.py 실행 중, 로봇 auto 모드.
설정: ~/HamdEyeCal/handeye_config.json (캘리브 경로 · TCP 길이 · 오프셋 · 최저 높이)

실행:
  python3 object_picker.py --dry                 # 좌표만 계산·출력 (로봇 안 움직임)
  python3 object_picker.py                       # 라벨 입력 → 집어서 place 에 놓기
  python3 object_picker.py --place place2 --grip-depth 15
"""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import String

import DR_init

CONFIG_PATH = Path("~/HamdEyeCal/handeye_config.json").expanduser()
POSES_PATH = Path(__file__).resolve().parent / "poses.json"
READY = (0, 0, 90, 0, 90, 0)
VEL = 20
REACH_MIN, REACH_MAX = 150.0, 850.0

ap = argparse.ArgumentParser()
ap.add_argument("--dry", action="store_true", help="좌표만 출력, 로봇 이동 없음")
ap.add_argument("--place", default="place", help="poses.json 의 놓을 자세 이름")
ap.add_argument("--grip-depth", type=float, default=20.0, help="물체 윗면에서 몇 mm 아래를 잡을지")
ap.add_argument("--clearance", type=float, default=80.0, help="물체 윗면 위 접근 높이(mm)")
ap.add_argument("--current", type=int, default=300, help="잡는 힘 200~400")
ap.add_argument("--grip-wait", type=float, default=1.5, help="그리퍼 후 대기(초)")
ap.add_argument("--min-conf", type=float, default=0.5, help="이 신뢰도 미만은 무시")
args = ap.parse_args()

CFG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
T_BASE_CAM = np.load(CFG.get("calib_path") or str(Path("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy").expanduser()))
OFFSET = np.array([CFG.get("offset_x_mm", 0.0), CFG.get("offset_y_mm", 0.0), CFG.get("offset_z_mm", 0.0)])
TCP_Z = float(CFG.get("tcp_z_mm", 200.0))
Z_MIN = float(CFG.get("z_min_mm", 50.0))
PLACE = json.loads(POSES_PATH.read_text()).get(args.place) if POSES_PATH.exists() else None


def cam_to_base_mm(xyz_m):
    return (T_BASE_CAM @ np.append(xyz_m, 1.0))[:3] * 1000.0 + OFFSET


# ── 검출 토픽 구독 (별도 노드·스레드: DSR_ROBOT2 가 자기 노드를 spin 하므로 분리) ──
latest = {"msg": None}
rclpy.init()
sub_node = rclpy.create_node("object_picker_sub")
sub_node.create_subscription(String, "/detections", lambda m: latest.update(msg=json.loads(m.data)), 10)
ex = SingleThreadedExecutor()
ex.add_node(sub_node)
threading.Thread(target=ex.spin, daemon=True).start()


def fresh_detections(after, timeout=3.0):
    """after(초) 이후에 찍힌 검출 결과를 기다려 반환. 없으면 None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = latest["msg"]
        if m and m["stamp"] > after:
            return m["objects"]
        time.sleep(0.05)
    return None


def choose(objects, label):
    cands = [o for o in objects if o["label"] == label and o["conf"] >= args.min_conf and o["xyz"]]
    return max(cands, key=lambda o: o["conf"]) if cands else None


# ── 로봇 ────────────────────────────────────────────────
DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "e0509"
node = rclpy.create_node("object_picker", namespace="dsr01")
DR_init.__dsr__node = node
import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, movejx, posj, posx, get_current_posx,  # noqa: E402
                        get_robot_mode, check_motion, DR_BASE, DR_MV_MOD_ABS)


def wait_idle(timeout=30.0):
    """movej/movel 응답이 도착 전에 올 수 있어서, check_motion()==0(IDLE)이 될 때까지 기다림."""
    t0 = time.time()
    time.sleep(0.2)
    while check_motion() != 0:
        if time.time() - t0 > timeout:
            raise RuntimeError("이동이 끝나지 않음 (시간 초과)")
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


def go_ready():
    movej(posj(*READY), vel=VEL, acc=VEL)
    wait_idle()
    return time.time()


def pick_and_place(target, gripper_open, gripper_close):
    tip, pose = tip_now()
    top = target[2]
    safe_z = max(tip[2], top + args.clearance)
    grip_z = max(top - args.grip_depth, Z_MIN)
    print(f"  물체 위로 (z={safe_z:.0f})"); move_tip([target[0], target[1], safe_z], pose[3:6])
    print("  그리퍼 열기");            gripper_open(); time.sleep(args.grip_wait)
    print(f"  하강 (z={grip_z:.0f})");  move_tip([target[0], target[1], grip_z], pose[3:6])
    print("  잡기");                    gripper_close(current=args.current); time.sleep(args.grip_wait)
    print("  상승");                    move_tip([target[0], target[1], safe_z], pose[3:6])
    if PLACE:
        above = list(PLACE["posx"]); above[2] += 100
        print(f"  '{args.place}' 로 이동")
        movejx(posx(above), vel=VEL, acc=VEL, ref=DR_BASE, sol=PLACE["sol"]); wait_idle()
        movel(posx(PLACE["posx"]), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS); wait_idle()
        print("  놓기");                gripper_open(); time.sleep(args.grip_wait)
        movel(posx(above), vel=VEL, acc=VEL, ref=DR_BASE, mod=DR_MV_MOD_ABS); wait_idle()
    else:
        print(f"  ⚠️  '{args.place}' 자세 없음 — 들고 있는 채로 준비자세로 갑니다")


try:
    if not args.dry:   # dry 는 좌표 계산만 하므로 bringup 없이도 동작
        assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), "로봇 서비스 없음 — bringup/도메인 확인"
    print(f"캘리브 적용 · 오프셋 {OFFSET} · TCP {TCP_Z:.0f}mm · 최저 {Z_MIN:.0f}mm · "
          f"place {'있음' if PLACE else '없음'}{' · DRY' if args.dry else ''}")
    if not args.dry:
        from dsr_gripper import gripper_open, gripper_close
        assert get_robot_mode() == 1, "auto 모드가 아닙니다"
        input("⚠️  준비자세로 이동합니다 (카메라 시야를 비우는 자세). 주변 확인 후 Enter ")
        ready_t = go_ready()
    else:
        ready_t = 0.0
    while rclpy.ok():
        objs = fresh_detections(ready_t + 0.3)
        if objs is None:
            print("검출 결과가 안 들어옵니다 — detector_node.py 실행 여부 확인")
            time.sleep(1.0)
            continue
        seen = sorted({o["label"] for o in objs})
        req = input(f"\n보이는 물체: {seen or '없음'}\n집을 물체 라벨 (q=종료, Enter=새로고침) > ").strip()
        if req == "q":
            break
        if not req:
            continue
        o = choose(fresh_detections(time.time()) or [], req)
        if o is None:
            print(f"'{req}' 를 찾지 못했습니다 (신뢰도≥{args.min_conf}, 유효 깊이 필요)")
            continue
        target = cam_to_base_mm(o["xyz"])
        r = float(np.hypot(*target[:2]))
        print(f"🎯 {req} conf={o['conf']:.2f} → base x={target[0]:.0f} y={target[1]:.0f} 윗면 z={target[2]:.0f} mm"
              f" (수평거리 {r:.0f}mm)")
        if not REACH_MIN < r < REACH_MAX:
            print("도달범위 밖 — 건너뜀")
            continue
        if args.dry:
            continue
        if input("집을까요? (y/N) ").strip().lower() != "y":
            continue
        pick_and_place(target, gripper_open, gripper_close)
        ready_t = go_ready()
        after = fresh_detections(ready_t + 0.3) or []
        left = [a for a in after if a["label"] == req and a["xyz"]
                and np.linalg.norm(cam_to_base_mm(a["xyz"])[:2] - target[:2]) < 40]
        print("✅ 완료" if not left else "⚠️  원래 자리에 아직 물체가 보입니다 — 집기 실패 가능")
except KeyboardInterrupt:
    print("\n중단됨 — 로봇이 멈추지 않으면 비상정지!")
except (RuntimeError, AssertionError) as e:
    print(f"\n⛔ {e}")
finally:
    ex.shutdown(timeout_sec=1.0)
    sub_node.destroy_node()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
