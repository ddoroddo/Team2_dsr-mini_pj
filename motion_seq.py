#!/usr/bin/env python3
"""E0509 연속 자세 녹화 / 재생 — 동작(action) 이름 하나에 여러 자세 + 그리퍼 동작을 순서대로 저장.

저장 파일: 같은 폴더의 sequences.json
  { "pick_box": [ {"posj":[..], "posx":[..], "sol":0, "move":"j", "grip":"open", "wait":0.0}, ... ] }

── 녹화 (직접교시) ─────────────────────────────────────────
  ※ 직접교시 중에는 gripper_service 를 꺼 두세요 (컨트롤러 DRL 이 돌면 직접교시가 막힘).
     그리퍼 동작은 '기록만' 해 두고, 재생할 때 실행됩니다.

  python3 motion_seq.py record pick_box        # 대화형 녹화 (manual 모드로 자동 전환)
      Enter = 현재 자세 추가            o = 추가 + 도착 후 그리퍼 열기
      c = 추가 + 도착 후 그리퍼 닫기     l = 다음 자세는 직선(movel)으로 이동
      u = 마지막 자세 취소              s = 목록 보기          q = 저장하고 종료(auto 복귀)

  python3 motion_seq.py add pick_box --grip close   # 한 줄씩 추가하는 방식도 가능

── 확인 / 편집 ────────────────────────────────────────────
  python3 motion_seq.py list                   # 동작 이름 목록
  python3 motion_seq.py list pick_box          # 자세 목록
  python3 motion_seq.py del pick_box 3         # 3번 자세 삭제
  python3 motion_seq.py clear pick_box         # 동작 통째로 삭제 (sequences.json.bak 백업)

── 재생 (auto 모드 + gripper_service 실행 중) ──────────────
  python3 motion_seq.py run pick_box           # 1회
  python3 motion_seq.py run pick_box -n 3      # 3회 반복
  python3 motion_seq.py run pick_box --dry     # 움직이지 않고 순서만 출력
  python3 motion_seq.py run pick_box --grip-wait 2   # 그리퍼 후 2초 대기 (기본 1.5)
  ※ 각 자세는 '도착 확인(모션 종료 + 관절각 0.5° 이내)' 후에만 그리퍼가 움직입니다.
  python3 motion_seq.py go pick_box 2          # 2번 자세 하나로만 이동 (확인용)
"""
import argparse
import json
import time
from pathlib import Path

FILE = Path(__file__).resolve().parent / "sequences.json"


def load():
    return json.loads(FILE.read_text()) if FILE.exists() else {}


def save(data):
    FILE.write_text(json.dumps(data, indent=2, ensure_ascii=False))


def describe(i, s):
    grip = {"open": "→ 그리퍼 열기", "close": "→ 그리퍼 닫기"}.get(s.get("grip"), "")
    wait = f" (대기 {s['wait']}s)" if s.get("wait") else ""
    mv = "직선" if s.get("move") == "l" else "관절"
    return f"{i:2d}. [{mv}] posj={[round(v, 1) for v in s['posj']]} {grip}{wait}"


ap = argparse.ArgumentParser(description="연속 자세 녹화/재생")
sub = ap.add_subparsers(dest="cmd", required=True)
p = sub.add_parser("record"); p.add_argument("name")
p = sub.add_parser("add"); p.add_argument("name")
p.add_argument("--grip", choices=["open", "close"], help="도착 후 그리퍼 동작")
p.add_argument("--move", choices=["j", "l"], default="j", help="이 자세로 갈 때 j=관절, l=직선")
p.add_argument("--wait", type=float, default=0.0, help="도착(+그리퍼) 후 대기 초")
p = sub.add_parser("list"); p.add_argument("name", nargs="?")
p = sub.add_parser("del"); p.add_argument("name"); p.add_argument("index", type=int)
p = sub.add_parser("clear"); p.add_argument("name")
p = sub.add_parser("go"); p.add_argument("name"); p.add_argument("index", type=int)
p.add_argument("--vel", type=float, default=20.0)
p = sub.add_parser("run"); p.add_argument("name")
p.add_argument("-n", "--count", type=int, default=1)
p.add_argument("--vel", type=float, default=20.0, help="관절 속도 deg/s")
p.add_argument("--lvel", type=float, default=50.0, help="직선 속도 mm/s")
p.add_argument("--current", type=int, default=300, help="닫을 때 힘 200~400")
p.add_argument("--grip-wait", type=float, default=1.5, help="그리퍼 명령 후 다음 이동까지 대기(초)")
p.add_argument("--dry", action="store_true", help="움직이지 않고 순서만 출력")
p.add_argument("--yes", action="store_true", help="시작 확인 생략")
args = ap.parse_args()
data = load()

# ── 파일만 다루는 명령 (로봇 연결 불필요) ──────────────────
if args.cmd == "list":
    if not args.name:
        print({k: f"{len(v)}개 자세" for k, v in data.items()} or "저장된 동작 없음")
    else:
        for i, s in enumerate(data.get(args.name, []), 1):
            print(describe(i, s))
    raise SystemExit
if args.cmd == "del":
    seq = data.get(args.name, [])
    assert 1 <= args.index <= len(seq), f"번호 범위 1~{len(seq)}"
    print("삭제:", describe(args.index, seq.pop(args.index - 1)))
    save(data)
    raise SystemExit
if args.cmd == "clear":
    if FILE.exists():
        FILE.with_suffix(".json.bak").write_text(FILE.read_text())
    print(f"'{args.name}' 삭제 ({len(data.pop(args.name, []))}개 자세) — 백업 sequences.json.bak")
    save(data)
    raise SystemExit
if args.cmd == "run" and args.dry:
    for i, s in enumerate(data[args.name], 1):
        print(describe(i, s))
    raise SystemExit

# ── 로봇 연결 ─────────────────────────────────────────────
import rclpy  # noqa: E402
import DR_init  # noqa: E402

DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "e0509"
rclpy.init()
node = rclpy.create_node("motion_seq", namespace="dsr01")
DR_init.__dsr__node = node
import DSR_ROBOT2 as dsr  # noqa: E402
from DSR_ROBOT2 import (movej, movel, posj, posx, get_current_posj, get_current_posx,  # noqa: E402
                        set_robot_mode, get_robot_mode, wait, check_motion, DR_BASE, DR_MV_MOD_ABS,
                        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS)


def snapshot(grip=None, move="j", wait_s=0.0):
    x, sol = get_current_posx()
    return {"posj": [round(v, 3) for v in get_current_posj()], "posx": [round(v, 3) for v in x],
            "sol": int(sol), "move": move, "grip": grip, "wait": wait_s}


def goto(s, vel, lvel):
    if s.get("move") == "l":
        movel(posx(s["posx"]), vel=lvel, acc=lvel, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    else:
        movej(posj(*s["posj"]), vel=vel, acc=vel)


def wait_arrival(target_posj, timeout=30.0, tol=0.5):
    """movej/movel 응답이 도착 전에 올 수 있어서, 모션 종료 + 목표 관절각 도달을 직접 확인."""
    t0 = time.time()
    time.sleep(0.2)                                   # 모션이 시작될 틈
    while check_motion() != 0:                        # 0=IDLE, 1=계산 중, 2=동작 중
        if time.time() - t0 > timeout:
            raise RuntimeError("이동이 끝나지 않음 (시간 초과) — 그리퍼 동작을 하지 않고 멈춥니다")
        time.sleep(0.1)
    while True:
        err = max(abs(a - b) for a, b in zip(get_current_posj(), target_posj))
        if err <= tol:
            return
        if time.time() - t0 > timeout:
            raise RuntimeError(f"목표 자세 미도달 (최대 오차 {err:.2f}°) — 그리퍼 동작을 하지 않고 멈춥니다")
        time.sleep(0.1)


try:
    assert dsr._ros2_get_robot_mode.wait_for_service(timeout_sec=10.0), \
        "로봇 서비스 없음 — bringup 실행 여부와 ROS_DOMAIN_ID(40) 확인"

    if args.cmd == "add":
        data.setdefault(args.name, []).append(snapshot(args.grip, args.move, args.wait))
        save(data)
        print(describe(len(data[args.name]), data[args.name][-1]))

    elif args.cmd == "record":
        seq = data.setdefault(args.name, [])
        set_robot_mode(ROBOT_MODE_MANUAL); wait(0.5)
        print(f"'{args.name}' 녹화 시작 (기존 {len(seq)}개 뒤에 이어서). 모드={get_robot_mode()} (0=manual)")
        print("직접교시 버튼으로 옮기고, 버튼에서 손을 뗀 뒤 키 입력:")
        print("  Enter=추가  o=추가+열기  c=추가+닫기  l=다음은 직선이동  u=취소  s=목록  q=저장/종료")
        next_move = "j"
        while True:
            k = input(f"[{len(seq)}개] > ").strip().lower()
            if k in ("", "o", "c"):
                seq.append(snapshot({"o": "open", "c": "close"}.get(k), next_move))
                save(data)
                print("  추가:", describe(len(seq), seq[-1]))
                next_move = "j"
            elif k == "l":
                next_move = "l"; print("  다음 자세는 직선(movel)으로 이동합니다")
            elif k == "u" and seq:
                print("  취소:", describe(len(seq), seq.pop())); save(data)
            elif k == "s":
                for i, s in enumerate(seq, 1):
                    print(" ", describe(i, s))
            elif k == "q":
                break
        set_robot_mode(ROBOT_MODE_AUTONOMOUS); wait(0.5)
        print(f"저장 완료: {len(seq)}개 자세. 모드={get_robot_mode()} (1=auto)")

    elif args.cmd == "go":
        s = data[args.name][args.index - 1]
        assert get_robot_mode() == 1, "auto 모드가 아닙니다"
        input(f"⚠️  {describe(args.index, s)} 로 이동. Enter (중단 Ctrl+C) ")
        movej(posj(*s["posj"]), vel=args.vel, acc=args.vel)
        wait_arrival(s["posj"])
        print("도착 (go 는 그리퍼 동작을 하지 않습니다)")

    elif args.cmd == "run":
        seq = data.get(args.name)
        assert seq, f"'{args.name}' 없음. 저장된 동작: {list(data)}"
        assert get_robot_mode() == 1, "auto 모드가 아닙니다 (녹화를 q 로 끝내면 자동 복귀)"
        if any(s.get("grip") for s in seq):
            from dsr_gripper import gripper_open, gripper_close
            from dsr_gripper.gripper_api import _client
            assert _client().wait_for_service(timeout_sec=3.0), \
                "gripper_service 가 없습니다 → ros2 run dsr_gripper gripper_service"
        for i, s in enumerate(seq, 1):
            print(describe(i, s))
        if not args.yes:
            input(f"⚠️  위 {len(seq)}개 자세를 {args.count}회 재생합니다. 주변 확인·비상정지 준비 후 Enter ")
        for n in range(1, args.count + 1):
            print(f"── {n}/{args.count} ──")
            for i, s in enumerate(seq, 1):
                print(" ", describe(i, s))
                print("     이동 중…", end="", flush=True)
                # 첫 자세는 어디서 출발할지 모르므로 항상 관절이동
                goto(s if i > 1 else {**s, "move": "j"}, args.vel, args.lvel)
                wait_arrival(s["posj"])
                print(" 도착")
                if s.get("grip") in ("open", "close"):
                    if s["grip"] == "open":
                        print("     그리퍼 열기"); gripper_open()
                    else:
                        print("     그리퍼 닫기"); gripper_close(current=args.current)
                    print(f"     그리퍼 대기 {args.grip_wait}s"); time.sleep(args.grip_wait)
                if s.get("wait"):
                    time.sleep(s["wait"])
        print("완료")
except KeyboardInterrupt:
    print("\n중단됨 — 로봇이 멈추지 않으면 비상정지!")
except RuntimeError as e:
    print(f"\n⛔ {e}")
finally:
    node.destroy_node()
    rclpy.shutdown()
