#!/usr/bin/env python3
"""물체 검출 노드 (임시: COCO 사전학습 YOLO) — 팀원 모듈로 교체할 부분.

카메라(RealSense)를 이 프로그램이 열고, 검출한 물체의 '카메라 좌표계 3D 위치(m)'를
ROS2 토픽 /detections (std_msgs/String, JSON) 로 계속 발행한다.
로봇 쪽(object_picker.py)은 이 토픽만 본다 → 같은 형식만 지키면 어떤 검출기로도 교체 가능.

── 토픽 형식 (교체 시 이 형식만 지키면 됨) ──────────────────────
  topic : /detections      type : std_msgs/msg/String   data : JSON 문자열
  {
    "stamp": 1790050000.123,            # time.time() — 이 프레임을 찍은 시각(초)
    "objects": [
      {"label": "cup",                  # 클래스 이름
       "conf": 0.91,                    # 신뢰도 0~1
       "bbox": [x1, y1, x2, y2],        # 컬러 이미지 픽셀 (표시용)
       "xyz": [0.031, -0.052, 0.781]},  # 카메라 좌표계 3D (m) = 물체 윗면 중심. 깊이 없으면 null
      ...
    ]
  }
  카메라 좌표계: RealSense 컬러 광학 좌표계 (x 오른쪽, y 아래, z 앞) — rs2_deproject_pixel_to_point 결과 그대로.

실행 (venv 에 ultralytics 설치됨):
  ~/venv/yolo/bin/python detector_node.py --show          # 검출 창 보면서 발행
  ~/venv/yolo/bin/python detector_node.py --classes cup bottle
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from std_msgs.msg import String
from ultralytics import YOLO

CONFIG_PATH = Path("~/HamdEyeCal/handeye_config.json").expanduser()
DEPTH_MIN, DEPTH_MAX = 0.2, 1.5

ap = argparse.ArgumentParser()
ap.add_argument("--weights", default=str(Path(__file__).resolve().parent / "models" / "yolo11n.pt"),
                help="YOLO 가중치 (없으면 자동 다운로드)")
ap.add_argument("--conf", type=float, default=0.4, help="최소 신뢰도")
ap.add_argument("--rate", type=float, default=5.0, help="발행 주기(Hz)")
ap.add_argument("--classes", nargs="*", help="이 라벨만 발행 (예: cup bottle)")
ap.add_argument("--show", action="store_true", help="검출 결과 창 표시")
args = ap.parse_args()


def start_camera():
    serial = ""
    try:
        serial = json.loads(CONFIG_PATH.read_text(encoding="utf-8")).get("camera_serial", "")
    except OSError:
        pass
    pipe, cfg = rs.pipeline(), rs.config()
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    for _ in range(15):                      # 자동노출 안정화
        align.process(pipe.wait_for_frames())
    return pipe, align


def bbox_to_xyz(bbox, depth):
    """bbox 가운데 40% 영역의 유효 깊이 중 '가까운 쪽'(하위 30%) 중앙값 → 물체 윗면. 3D 는 bbox 중심 기준."""
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    hw, hh = max(2, (x2 - x1) * 0.2), max(2, (y2 - y1) * 0.2)
    d = np.asanyarray(depth.get_data())[int(cy - hh):int(cy + hh), int(cx - hw):int(cx + hw)]
    z = d[d > 0].astype(np.float32) * depth.get_units()
    z = z[(z >= DEPTH_MIN) & (z <= DEPTH_MAX)]
    if z.size < 10:
        return None
    z_top = float(np.median(z[z <= np.percentile(z, 30)]))
    intr = depth.profile.as_video_stream_profile().get_intrinsics()
    return [round(v, 4) for v in rs.rs2_deproject_pixel_to_point(intr, [float(cx), float(cy)], z_top)]


rclpy.init()
node = rclpy.create_node("yolo_detector")
pub = node.create_publisher(String, "/detections", 10)
Path(args.weights).parent.mkdir(parents=True, exist_ok=True)
model = YOLO(args.weights)
pipe, align = start_camera()
print(f"검출 시작: {Path(args.weights).name}, conf≥{args.conf}, {args.rate}Hz → /detections"
      + (f", 라벨 {args.classes}" if args.classes else ""))
period = 1.0 / args.rate
try:
    while rclpy.ok():
        t0 = time.time()
        f = align.process(pipe.wait_for_frames())
        color, depth = np.asanyarray(f.get_color_frame().get_data()), f.get_depth_frame()
        stamp = time.time()
        res = model(color, conf=args.conf, verbose=False)[0]
        objs = []
        for b in res.boxes:
            label = res.names[int(b.cls)]
            if args.classes and label not in args.classes:
                continue
            bbox = [round(float(v), 1) for v in b.xyxy[0].tolist()]
            objs.append({"label": label, "conf": round(float(b.conf), 3), "bbox": bbox,
                         "xyz": bbox_to_xyz(bbox, depth)})
        pub.publish(String(data=json.dumps({"stamp": stamp, "objects": objs})))
        if args.show:
            vis = color.copy()
            for o in objs:
                x1, y1, x2, y2 = map(int, o["bbox"])
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                z = f" z={o['xyz'][2]:.2f}m" if o["xyz"] else " no depth"
                cv2.putText(vis, f"{o['label']} {o['conf']:.2f}{z}", (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("detector (q=quit)", vis)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        time.sleep(max(0.0, period - (time.time() - t0)))
except KeyboardInterrupt:
    pass
finally:
    pipe.stop()
    cv2.destroyAllWindows()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
