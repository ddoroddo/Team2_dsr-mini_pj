#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=====================================================================
  ROS 2 Real-Time Burger Ingredients YOLOv8 Detector Node
=====================================================================
1. NVIDIA RTX 5080 GPU 기반 초고속 YOLOv8n 추론 (~2.5ms)
2. RealSense D455F RGB-D 정렬 뎁스 융합으로 정밀 3D 좌표 산출
3. Hand-Eye 캘리브레이션 행렬을 통한 로봇 Base 3D 좌표 실시간 계산
4. /home/omen/doosan_ws/latest_targets.json 자동 출력 (로봇 제어와 완벽 연동)
5. /camera/detected_image 로 시각화 영상 퍼블리시
=====================================================================
"""

import os
import sys
import json
import time
from pathlib import Path

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ultralytics import YOLO

# Model and outputs
MODEL_PATH = Path("/home/omen/doosan_ws/yolo_runs/burger_yolov8n/weights/best.pt")
TARGETS_JSON = Path("/home/omen/doosan_ws/latest_targets.json")
CALIB_PATH = Path("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy").expanduser()


class YOLOBurgerDetector(Node):
    def __init__(self):
        super().__init__("yolo_burger_detector")

        # 1. Load YOLOv8 Model
        self.get_logger().info(f"Loading YOLOv8 weights from: {MODEL_PATH}")
        self.model = YOLO(str(MODEL_PATH))
        self.get_logger().info(f"YOLOv8 Model loaded successfully! Classes: {self.model.names}")

        # 2. Camera Intrinsics
        self.fx = 386.35
        self.fy = 385.76
        self.cx = 321.34
        self.cy = 244.53
        self.intrinsics_ready = False

        # 3. Load Hand-Eye Calibration
        self.T_base_cam = None
        if CALIB_PATH.exists():
            try:
                self.T_base_cam = np.load(str(CALIB_PATH))
                self.get_logger().info(f"Hand-Eye Calibration Loaded: {CALIB_PATH}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load calibration: {e}")
        else:
            self.get_logger().warn(f"Calibration file {CALIB_PATH} not found.")

        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None

        # 4. Subscribers & Publisher
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.info_cb, 10
        )
        self.sub_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_cb, 10
        )
        self.sub_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_cb, 10
        )
        self.pub_annotated = self.create_publisher(Image, "/camera/detected_image", 10)

        self.show_gui = True
        self._last_warn_time = 0.0

        # 15Hz Detection Loop Timer
        self.timer = self.create_timer(0.066, self.detect_loop)
        self.get_logger().info("YOLOBurgerDetector initialized and ready!")

    def info_cb(self, msg: CameraInfo):
        if not self.intrinsics_ready:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_ready = True
            self.get_logger().info(f"Camera Intrinsics updated: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    def color_cb(self, msg: Image):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            pass

    def extract_box_yaw(self, crop_img):
        """
        [Phase 1 & Phase 2] 박스 ROI 영상에서 OpenCV minAreaRect로 기울기 각도를 산출하고,
        Hand-Eye 행렬을 통해 로봇 Base 기준 Yaw 각도로 변환합니다.
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
            r_mat = self.T_base_cam[:3, :3] if self.T_base_cam is not None else np.eye(3)
            v_base = r_mat @ v_cam
            yaw_base = math.degrees(math.atan2(v_base[1], v_base[0]))

            # 180도 대칭성 정규화 (케이블 꼬임 및 조인트 리밋 방지: [-90, +90])
            opt_yaw = (yaw_base + 90.0) % 180.0 - 90.0
            return round(float(opt_yaw), 1)
        except Exception:
            return 0.0

    def detect_loop(self):
        if self.latest_color is None or self.latest_depth is None:
            if getattr(self, "show_gui", True):
                wait_img = np.zeros((480, 640, 3), dtype=np.uint8)
                c_ok = self.latest_color is not None
                d_ok = self.latest_depth is not None
                cv2.putText(wait_img, "YOLO Detector - Waiting for Camera...", (30, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(wait_img, f"Color Stream: {'OK' if c_ok else 'Waiting...'}", (50, 220),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if c_ok else (0, 0, 255), 2)
                cv2.putText(wait_img, f"Depth Stream: {'OK' if d_ok else 'Waiting...'}", (50, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if d_ok else (0, 0, 255), 2)
                cv2.putText(wait_img, "Expected topics:", (50, 320),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                cv2.putText(wait_img, "- /camera/camera/color/image_raw", (70, 350),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                cv2.putText(wait_img, "- /camera/camera/aligned_depth_to_color/image_raw", (70, 380),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                try:
                    cv2.imshow("YOLO Real-time Detection (Press 'q' in window to close GUI)", wait_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.show_gui = False
                        cv2.destroyAllWindows()
                except Exception as e:
                    self.get_logger().error(f"cv2.imshow GUI error: {e}")
                    self.show_gui = False

            now = time.time()
            if now - self._last_warn_time > 3.0:
                self.get_logger().warn(
                    f"Waiting for camera frames... [Color: {'OK' if self.latest_color is not None else 'None'}, "
                    f"Depth: {'OK' if self.latest_depth is not None else 'None'}]"
                )
                self._last_warn_time = now
            return

        color_img = self.latest_color.copy()
        depth_img = self.latest_depth.copy()
        img_h, img_w = color_img.shape[:2]

        # Run GPU inference
        results = self.model.predict(color_img, conf=0.45, device=0, verbose=False)
        targets_dict = {}
        annotated = color_img.copy()

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                # Clamp to frame
                x1 = max(0, min(img_w - 1, x1))
                x2 = max(0, min(img_w - 1, x2))
                y1 = max(0, min(img_h - 1, y1))
                y2 = max(0, min(img_h - 1, y2))

                if x2 - x1 < 15 or y2 - y1 < 15:
                    continue

                # Center pixel (u, v)
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2

                # Sample depth around center (5x5 median patch)
                patch = depth_img[max(0, v - 3):min(img_h, v + 4), max(0, u - 3):min(img_w, u + 4)]
                valid_depths = patch[patch > 0]
                if len(valid_depths) == 0:
                    depth_mm = float(depth_img[v, u]) if depth_img[v, u] > 0 else 920.0
                else:
                    depth_mm = float(np.median(valid_depths))

                if depth_mm < 300 or depth_mm > 1500:
                    depth_mm = 920.0  # Fallback to typical desk distance

                # 3D Coordinates in Camera Optical Frame (mm)
                xc = (u - self.cx) * depth_mm / self.fx
                yc = (v - self.cy) * depth_mm / self.fy
                zc = depth_mm

                # 3D Coordinates in Robot Base Frame (mm)
                bx, by, bz = 0.0, 0.0, 0.0
                if self.T_base_cam is not None:
                    p_cam = np.array([xc / 1000.0, yc / 1000.0, zc / 1000.0, 1.0])
                    p_base = self.T_base_cam @ p_cam
                    bx = float(p_base[0] * 1000.0)
                    by = float(p_base[1] * 1000.0)
                    bz = float(p_base[2] * 1000.0)

                # [Phase 1 & Phase 2] Box Orientation Extraction
                crop = color_img[y1:y2, x1:x2]
                yaw_deg = self.extract_box_yaw(crop)

                # Store highest confidence detection for each class
                if cls_name not in targets_dict or conf > targets_dict[cls_name]["conf"]:
                    targets_dict[cls_name] = {
                        "conf": conf,
                        "base_x": bx,
                        "base_y": by,
                        "base_z": bz,
                        "yaw_deg": float(yaw_deg),
                        "cam_x": xc,
                        "cam_y": yc,
                        "depth_mm": int(depth_mm),
                        "pixel_uv": [u, v]
                    }

                # Visualization overlay
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(annotated, (u, v), 4, (0, 0, 255), -1)
                label_txt = f"{cls_name} {conf:.2f} ({yaw_deg:+.0f}deg) [X:{bx:.0f} Y:{by:.0f}]"
                cv2.putText(annotated, label_txt, (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # Save to latest_targets.json
        if targets_dict:
            try:
                # Remove internal 'conf' key for clean JSON export
                export_dict = {}
                for k, v in targets_dict.items():
                    d = dict(v)
                    del d["conf"]
                    export_dict[k] = d
                with open(TARGETS_JSON, "w") as f:
                    json.dump(export_dict, f, indent=2)
            except Exception:
                pass

        # FPS and Inference Speed Display
        fps = 1.0 / max(1e-4, time.time() - getattr(self, "_last_time", time.time() - 0.033))
        self._last_time = time.time()
        cv2.putText(annotated, f"YOLOv8n GPU | FPS: {fps:.1f} | Detections: {len(targets_dict)}",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)

        # Save latest frame snapshot for quick inspection
        try:
            cv2.imwrite("/home/omen/doosan_ws/latest_yolo_view.jpg", annotated)
        except Exception:
            pass

        # Publish annotated ROS image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.pub_annotated.publish(out_msg)
        except Exception:
            pass

        # Live OpenCV GUI Window (if DISPLAY is available)
        if getattr(self, "show_gui", True):
            try:
                cv2.imshow("YOLO Real-time Detection (Press 'q' in window to close GUI)", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.show_gui = False
                    cv2.destroyAllWindows()
            except Exception as e:
                self.get_logger().error(f"cv2.imshow GUI error: {e}")
                self.show_gui = False


def main():
    import argparse
    parser = argparse.ArgumentParser(description="YOLO Burger Detector Node")
    parser.add_argument("--no-gui", action="store_true", help="화면 창(cv2.imshow) 없이 백그라운드로 실행")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YOLOBurgerDetector()
    node.show_gui = not args.no_gui
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
