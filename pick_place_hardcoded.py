#!/usr/bin/env python3
"""E0509 하드코딩 픽앤플레이스 — 직접교시로 저장한 두 자세(pick / place)를 반복.

사전 조건: bringup + `ros2 run dsr_gripper gripper_service` 실행 중.

1) 자세 가르치기 (그리퍼 끝이 물체를 '잡는 높이'에 있을 때 저장)
     python3 pick_place_hardcoded.py mode manual   # 직접교시 버튼으로 팔 이동 가능
     (팔을 집을 위치로 옮긴 뒤)  python3 pick_place_hardcoded.py teach pick
     (팔을 놓을 위치로 옮긴 뒤)  python3 pick_place_hardcoded.py teach place
     python3 pick_place_hardcoded.py mode auto     # 프로그램 동작 전 반드시 auto 복귀
2) 확인 / 실행
     python3 pick_place_hardcoded.py show
     python3 pick_place_hardcoded.py run -n 1       # 1회 (--approach 100 = 위에서 100mm 접근)
3) 여러 위치 / 정리
     python3 pick_place_hardcoded.py teach place2              # 이름은 자유
     python3 pick_place_hardcoded.py run --place place2        # 다른 이름으로 실행
     python3 pick_place_hardcoded.py delete place2             # 하나 지우기
     python3 pick_place_hardcoded.py reset                     # 전부 지우기 (poses.json.bak 백업)

동작 순서: 준비자세 → 그리퍼 열기 → pick 위 → 하강 → 닫기 → 상승
          → place 위 → 하강 → 열기 → 상승 → 준비자세
"""
import argparse
import json
import time
from pathlib import Path

import rclpy
import DR_init

POSES = Path(__file__).resolve().parent / "poses.json"


def load():
    return json.loads(POSES.read_text()) if POSES.exists() else {}

READY = [0, 0, 90, 0, 90, 0]

ap = argparse.ArgumentParser()
sub = ap.add_subparsers(dest="cmd", required=True)
sub.add_parser("show")
sub.add_parser("reset")
dl = sub.add_parser("delete"); dl.add_argument("name")
t = sub.add_parser("teach"); t.add_argument("name", help="저장 이름 (pick / place)")
m = sub.add_parser("mode"); m.add_argument("which", choices=["manual", "auto"])
r = sub.add_parser("run")
r.add_argument("-n", "--count", type=int, default=1)
r.add_argument("--approach", type=float, default=100.0, help="접근/상승 높이(mm)")
r.add_argument("--vel", type=float, default=30.0, help="직선 속도(mm/s), 관절은 20deg/s 고정")
r.add_argument("--grip", type=int, default=300, help="잡는 힘(전류) 200~400")
r.add_argument("--yes", action="store_true", help="시작 확인 생략")
r.add_argument("--grip-wait", type=float, default=1.5, help="그리퍼 명령 후 다음 이동까지 대기(초)")
r.add_argument("--pick", default="pick", help="집을 자세 이름 (기본 pick)")
r.add_argument("--place", default="place", help="놓을 자세 이름 (기본 place)")
args = ap.parse_args()

if args.cmd in ("reset", "delete"):
    poses = load()
    if args.cmd == "reset":
        if POSES.exists():
            POSES.with_suffix(".json.bak").write_text(POSES.read_text())
            POSES.unlink()
        print("전부 삭제 (백업: poses.json.bak)")
    elif poses.pop(args.name, None) is None:
        print(f"'{args.name}' 없음. 저장된 이름: {list(poses)}")
    else:
        POSES.write_text(json.dumps(poses, indent=2))
        print(f"'{args.name}' 삭제. 남은 이름: {list(poses)}")
    raise SystemExit

DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "e0509"
rclpy.init()
node = rclpy.create_node("pick_place_hardcoded", namespace="dsr01")
DR_init.__dsr__node = node

import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, movejx, posj, posx, get_current_posj,  # noqa: E402
                        get_current_posx, set_robot_mode, get_robot_mode, wait, check_motion,
                        ROBOT_MODE_AUTONOMOUS, ROBOT_MODE_MANUAL, DR_BASE, DR_MV_MOD_ABS)


def lifted(p, dz):
    q = list(p); q[2] += dz
    return posx(q)


def step(msg):
    print(f"  → {msg}")


def wait_idle(timeout=30.0):
    """movej/movel 응답이 도착 전에 올 수 있어서, check_motion()==0(IDLE)이 될 때까지 기다림."""
    t0 = time.time()
    time.sleep(0.2)                                   # 모션이 시작될 틈
    while check_motion() != 0:                        # 0=IDLE, 1=계산 중, 2=동작 중
        if time.time() - t0 > timeout:
            raise RuntimeError("이동이 끝나지 않음 (시간 초과) — 그리퍼 동작을 하지 않고 멈춥니다")
        time.sleep(0.1)
    time.sleep(0.2)                                   # 정착


try:
    assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), \
        "로봇 서비스 없음 — bringup 실행 여부와 ROS_DOMAIN_ID(40) 확인"

    if args.cmd == "mode":
        set_robot_mode(ROBOT_MODE_MANUAL if args.which == "manual" else ROBOT_MODE_AUTONOMOUS)
        wait(0.5)
        print("현재 모드:", get_robot_mode(), "(0=manual, 1=auto)")

    elif args.cmd == "teach":
        x, sol = get_current_posx()
        poses = load()
        poses[args.name] = {"posj": [round(v, 3) for v in get_current_posj()],
                            "posx": [round(v, 3) for v in x], "sol": int(sol)}
        POSES.write_text(json.dumps(poses, indent=2))
        print(f"'{args.name}' 저장:", poses[args.name])

    elif args.cmd == "show":
        print("현재 posj:", [round(v, 1) for v in get_current_posj()])
        print("현재 posx:", [round(v, 1) for v in get_current_posx()[0]])
        for k, v in load().items():
            print(f"{k:6s} posx={v['posx']}  sol={v['sol']}")

    elif args.cmd == "run":
        from dsr_gripper import gripper_open, gripper_close
        poses = load()
        missing = {args.pick, args.place} - poses.keys()
        assert not missing, f"먼저 teach 하세요: {missing} (저장된 이름: {list(poses)})"
        pick, place = poses[args.pick], poses[args.place]
        assert get_robot_mode() == 1, "auto 모드가 아닙니다 → 'mode auto' 먼저 실행"
        V, A = args.vel, args.vel
        up = args.approach

        def goto_above(p):  # 관절보간으로 '위' 자세까지 (가르친 자세의 형상 sol 유지)
            movejx(lifted(p["posx"], up), vel=20, acc=20, ref=DR_BASE, sol=p["sol"])

        def down_up(p, action):
            movel(posx(p["posx"]), vel=V, acc=A, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            wait_idle()                        # 도착 확인 후에만 그리퍼
            action(); time.sleep(args.grip_wait)   # 그리퍼 동작 완료 대기
            movel(lifted(p["posx"], up), vel=V, acc=A, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        print(f"{args.pick} = {pick['posx']}\n{args.place} = {place['posx']}\n접근높이 {up}mm, 속도 {V}mm/s, {args.count}회")
        if not args.yes:
            input("⚠️  로봇이 움직입니다. 주변 확인·비상정지 준비 후 Enter (중단: Ctrl+C) ")

        movej(posj(*READY), vel=20, acc=20)
        wait_idle()
        gripper_open(); time.sleep(args.grip_wait)
        for i in range(1, args.count + 1):
            print(f"[{i}/{args.count}]")
            step("pick 위로");   goto_above(pick)
            step("집기");        down_up(pick, lambda: gripper_close(current=args.grip))
            step("place 위로");  goto_above(place)
            step("놓기");        down_up(place, gripper_open)
        movej(posj(*READY), vel=20, acc=20)
        print("완료")
except KeyboardInterrupt:
    print("\n중단됨 — 로봇이 멈추지 않으면 비상정지!")
except RuntimeError as e:
    print(f"\n⛔ {e}")
finally:
    node.destroy_node()
    rclpy.shutdown()
