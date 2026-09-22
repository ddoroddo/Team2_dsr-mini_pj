#!/usr/bin/env python3
"""E0509: 지정 자세로 이동한 뒤 그리퍼 열기/닫기를 반복.

사전 조건 (터미널별로, 모두 ~/doosan_ws/install/setup.bash source):
  1) bringup   : ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.68 model:=e0509
  2) 그리퍼    : ros2 run dsr_gripper gripper_service

사용 예:
  python3 gripper_repeat.py --show                       # 현재 관절각만 출력 (움직이지 않음)
  python3 gripper_repeat.py                              # 현재 자세 그대로, 5회 반복
  python3 gripper_repeat.py --posj 0 0 90 0 90 0 -n 10   # 준비자세로 이동 후 10회
"""
import argparse
import time

import rclpy
import DR_init

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"

ap = argparse.ArgumentParser()
ap.add_argument("--posj", type=float, nargs=6, metavar="J",
                help="이동할 관절각(deg) 6개. 생략하면 현재 자세에서 반복")
ap.add_argument("-n", "--count", type=int, default=5, help="열기/닫기 반복 횟수")
ap.add_argument("--interval", type=float, default=1.5, help="동작 사이 대기(초)")
ap.add_argument("--current", type=int, default=300, help="닫을 때 힘(전류), 200~400 권장")
ap.add_argument("--vel", type=float, default=20, help="movej 속도(deg/s)")
ap.add_argument("--show", action="store_true", help="현재 관절각만 출력하고 종료")
args = ap.parse_args()

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
rclpy.init()
node = rclpy.create_node("gripper_repeat", namespace=ROBOT_ID)
DR_init.__dsr__node = node

import DSR_ROBOT2 as dsr  # noqa: E402  (노드 등록 후 import 해야 함)
from DSR_ROBOT2 import movej, get_current_posj, set_robot_mode, posj, ROBOT_MODE_AUTONOMOUS  # noqa: E402
from dsr_gripper import gripper_open, gripper_close  # noqa: E402

try:
    assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), \
        "로봇 서비스 없음 — bringup 실행 여부와 ROS_DOMAIN_ID(40) 확인"
    print("현재 관절각(deg):", [round(v, 1) for v in get_current_posj()])
    if args.show:
        raise SystemExit

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    if args.posj:
        target = posj(*args.posj)
        input(f"⚠️  {list(args.posj)} 로 이동합니다. 주변 확인·비상정지 준비 후 Enter (중단: Ctrl+C) ")
        movej(target, vel=args.vel, acc=args.vel)
        print("이동 완료:", [round(v, 1) for v in get_current_posj()])

    for i in range(1, args.count + 1):
        print(f"[{i}/{args.count}] 열기");  gripper_open()
        time.sleep(args.interval)
        print(f"[{i}/{args.count}] 닫기");  gripper_close(current=args.current)
        time.sleep(args.interval)
    gripper_open()
    print("완료 (그리퍼 열린 상태로 종료)")
except KeyboardInterrupt:
    print("\n중단됨")
finally:
    node.destroy_node()
    rclpy.shutdown()
