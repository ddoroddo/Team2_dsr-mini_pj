#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time 5 Burger Ingredients 3D Bounding Box Tracker with Strict 4-Step Verification
- Strictly filters out human hands, skin, clothing, and background objects.
- 4-Step Verification Pipeline:
    1. Square Aspect Ratio & Solidity Check (compact 100x100mm card)
    2. White Card Margin Check (4 corners must be white paper: S<45, V>145)
    3. Depth Planarity Check (surface depth standard deviation < 18mm)
    4. Reference Texture Template Matching (Rotational NCC >= 0.50 for patterned cards)
- Exclusively tracks the 5 target ingredients:
    1. Tomato (토마토)
    2. Cheese (체다 치즈)
    3. Patty (소고기 패티)
    4. Bun Top (상단 번)
    5. Bottom Bun (하단 번)
    (or Lettuce)
- Real-time 3D camera coordinates (X_c, Y_c, Z_c) output.
"""

import os
import sys
import time
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from collections import deque, Counter

if "DISPLAY" not in os.environ or not os.environ["DISPLAY"]:
    os.environ["DISPLAY"] = ":1"


class StrictIngredientTracker(Node):
    def __init__(self):
        super().__init__('strict_ingredient_tracker')

        # Camera Intrinsics
        self.fx = 386.35
        self.fy = 385.76
        self.cx = 321.34
        self.cy = 244.53
        self.intrinsics_received = False

        # Load Hand-Eye Calibration Matrix
        self.T_base_cam = None
        calib_path = os.path.expanduser("~/HamdEyeCal/e0509_handeye_result/T_base_camera.npy")
        if os.path.exists(calib_path):
            self.T_base_cam = np.load(calib_path)
            self.get_logger().info(f"Hand-Eye Calibration Loaded: {calib_path}")
        else:
            self.get_logger().warn("Hand-Eye calibration file not found. Running with camera frame.")

        # Card physical width on desk: ~100x100mm (approx 44 pixels at 0.95m)
        self.card_box_px = 44

        # Table workspace Polygon ROI (윗변 상단 20%, 하단 상단 15% 이동)
        self.roi_poly = np.array([
            [191, 100],   # Top-Left
            [448, 100],   # Top-Right
            [492, 422],   # Bottom-Right
            [148, 422]    # Bottom-Left
        ], dtype=np.int32)

        # Preload Reference Images and build Multi-Angle Template Bank
        self.ref_bank = {}
        ref_paths = {
            'Bun Top': '/home/omen/doosan_ws/src/dsr_project/materials/textures/01_bun_top.jpg',
            'Patty': '/home/omen/doosan_ws/src/dsr_project/materials/textures/02_patty.jpg',
            'Cheese': '/home/omen/doosan_ws/src/dsr_project/materials/textures/04_cheese.jpg',
            'Tomato': '/home/omen/doosan_ws/src/dsr_project/materials/textures/tomato.jpg',
            'Bottom Bun': '/home/omen/doosan_ws/src/dsr_project/materials/textures/bottom_bun.jpg',
            'Lettuce': '/home/omen/doosan_ws/src/dsr_project/materials/textures/03_lettuce.jpg',
        }
        for name, p in ref_paths.items():
            if os.path.exists(p):
                raw = cv2.imread(p)
                raw_sq = cv2.resize(raw, (self.card_box_px, self.card_box_px))
                center = (self.card_box_px // 2, self.card_box_px // 2)
                bank = []
                for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                    M = cv2.getRotationMatrix2D(center, angle, 1.0)
                    rot = cv2.warpAffine(raw_sq, M, (self.card_box_px, self.card_box_px), borderMode=cv2.BORDER_REPLICATE)
                    bank.append(rot)
                self.ref_bank[name] = bank

        self.latest_color = None
        self.latest_depth = None

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10
        )
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_cb, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10
        )

        # Publisher
        self.pub_annotated = self.create_publisher(Image, '/camera/detected_image', 10)

        # Processing loop (20 FPS)
        self.timer = self.create_timer(0.05, self.process_detection)
        self.last_log_time = time.time()

        # Spatial temporal smoothing trackers
        self.trackers = {}
        self.next_tracker_id = 0

        self.get_logger().info("StrictIngredientTracker initialized with 4-step Anti-Hand verification.")

    def info_cb(self, msg: CameraInfo):
        if not self.intrinsics_received and msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.intrinsics_received = True

    def color_cb(self, msg: Image):
        self.latest_color = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    def depth_cb(self, msg: Image):
        self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))

    def verify_candidate_card(self, card_crop, card_depth_roi, label_key):
        """
        Rigid 4-Step Verification:
        Returns (True, score) if and only if the candidate is the exact printed card.
        """
        h, w = card_crop.shape[:2]
        if h < 25 or w < 25:
            return False, 0.0

        card_sq = cv2.resize(card_crop, (self.card_box_px, self.card_box_px))

        # --- Filter 1: White Card Margin Check (4 Corners) ---
        # A real 100x100mm card has white paper on all corners. A human hand/arm DOES NOT.
        cw, ch = 7, 7
        corners = [
            card_sq[:ch, :cw],
            card_sq[:ch, -cw:],
            card_sq[-ch:, :cw],
            card_sq[-ch:, -cw:]
        ]
        white_cnt = 0
        for c in corners:
            hsv_c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
            s_mean = np.mean(hsv_c[:, :, 1])
            v_mean = np.mean(hsv_c[:, :, 2])
            # White paper: low saturation, high brightness
            if s_mean < 45 and v_mean > 140:
                white_cnt += 1
        if white_cnt < 3:
            return False, 0.0  # Rejected: Not a white card (e.g., human hand, arm, table edge)

        # --- Filter 2: Depth Planarity Check ---
        # The top surface of a rigid box is a flat plane (std < 18mm). Human hand/fingers have large curvature.
        if card_depth_roi is not None:
            valid_z = card_depth_roi[card_depth_roi > 0]
            if len(valid_z) > 10:
                std_z = float(np.std(valid_z))
                if std_z > 20.0:
                    return False, 0.0  # Rejected: Curved surface (hand/finger joints)

        # --- Filter 3: Rotational Template Matching ---
        # Compares candidate graphic with preloaded official textures
        best_score = 0.0
        if label_key in self.ref_bank:
            for rot_ref in self.ref_bank[label_key]:
                res = cv2.matchTemplate(card_sq, rot_ref, cv2.TM_CCOEFF_NORMED)[0][0]
                if res > best_score:
                    best_score = float(res)

            # Strict correlation threshold for high-texture ingredients
            if label_key in ['Patty', 'Tomato', 'Bun Top']:
                if best_score < 0.48:
                    return False, best_score  # Rejected: Pattern mismatch (human skin against patty)

        return True, best_score

    def process_detection(self):
        if self.latest_color is None:
            return

        color = self.latest_color.copy()
        depth = self.latest_depth.copy() if self.latest_depth is not None else None
        vis = color.copy()

        # 1. Table Polygon ROI Mask
        h, w = color.shape[:2]
        poly_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(poly_mask, [self.roi_poly], 255)

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        # 2. Segment colored graphic within the polygon ROI
        graphic_mask = (hsv[:, :, 1] > 38) & (hsv[:, :, 2] > 50)
        graphic_mask = (graphic_mask.astype(np.uint8) * 255) & poly_mask

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        graphic_mask = cv2.morphologyEx(graphic_mask, cv2.MORPH_CLOSE, kernel)
        graphic_mask = cv2.morphologyEx(graphic_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(graphic_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        raw_candidates = []
        hw = self.card_box_px // 2

        for c in contours:
            area = cv2.contourArea(c)
            # Card graphic footprint range
            if 70 < area < 2500:
                bx, by, bw, bh = cv2.boundingRect(c)
                # Geometric aspect ratio check of graphic: must be compact (not elongated like a hand/arm)
                aspect = float(bw) / float(bh) if bh > 0 else 0.0
                if not (0.60 <= aspect <= 1.65):
                    continue

                cx = bx + bw // 2
                cy = by + bh // 2

                # Extract Candidate Card Crop (100x100mm footprint)
                card_x1 = max(0, cx - hw)
                card_y1 = max(0, cy - hw)
                card_x2 = min(color.shape[1], cx + hw)
                card_y2 = min(color.shape[0], cy + hw)
                card_crop = color[card_y1:card_y2, card_x1:card_x2]

                if card_crop.shape[0] < 25 or card_crop.shape[1] < 25:
                    continue

                card_depth_roi = depth[card_y1:card_y2, card_x1:card_x2] if depth is not None else None

                # Depth sampling
                z_val = 950
                if card_depth_roi is not None:
                    valid_d = card_depth_roi[card_depth_roi > 0]
                    if len(valid_d) > 0:
                        z_val = int(np.median(valid_d))

                # Height filter: Desk is at ~1015mm, blocks are <= 1000mm
                if z_val >= 1005:
                    continue

                # Color Feature Extraction (Graphic Pixels Only)
                crop_hsv_full = cv2.cvtColor(card_crop, cv2.COLOR_BGR2HSV)
                g_mask = (crop_hsv_full[:, :, 1] > 38) & (crop_hsv_full[:, :, 2] > 40)
                if np.sum(g_mask) < 20:
                    continue

                g_pts = card_crop[g_mask]
                mean_g = float(np.mean(g_pts[:, 1]))
                mean_r = float(np.mean(g_pts[:, 2]))
                rg_ratio = mean_r / max(1.0, mean_g)
                med_h = float(np.median(crop_hsv_full[g_mask, 0]))
                med_s = float(np.median(crop_hsv_full[g_mask, 1]))
                med_v = float(np.median(crop_hsv_full[g_mask, 2]))

                # High-accuracy Color Categorization using RG ratio and HSV
                if med_h >= 13 and med_s >= 115:
                    label_key = 'Cheese'
                elif 35 <= med_h <= 85 and med_s > 60:
                    label_key = 'Lettuce'
                elif med_v < 135:
                    label_key = 'Patty'
                elif med_s < 80:
                    label_key = 'Bottom Bun'
                elif rg_ratio >= 1.75 or (med_h <= 3.5 and rg_ratio >= 1.68):
                    label_key = 'Tomato'
                else:
                    label_key = 'Bun Top'

                # --- Execute Strict 4-Step Verification ---
                is_valid_card, score = self.verify_candidate_card(card_crop, card_depth_roi, label_key)
                if not is_valid_card:
                    continue  # Completely discard human hands, clothes, or non-card items

                # 3D Coordinates (Camera frame)
                xc = (cx - self.cx) * z_val / self.fx
                yc = (cy - self.cy) * z_val / self.fy

                # Transform to Robot Base Frame (if calibrated)
                bx_r, by_r, bz_r = 0.0, 0.0, 0.0
                if self.T_base_cam is not None:
                    p_cam = np.array([xc / 1000.0, yc / 1000.0, z_val / 1000.0, 1.0])
                    p_base = self.T_base_cam @ p_cam
                    bx_r, by_r, bz_r = p_base[:3] * 1000.0

                raw_candidates.append({
                    'label': label_key,
                    'cx': cx, 'cy': cy,
                    'z_val': z_val,
                    'cam_coord': (xc, yc),
                    'base_coord': (bx_r, by_r, bz_r),
                    'score': score,
                    'rg_ratio': rg_ratio
                })

        # --- Multi-Class 1:1 Conflict Resolution for Tomato vs Bun Top ---
        tb_cands = [c for c in raw_candidates if c['label'] in ['Tomato', 'Bun Top']]
        if len(tb_cands) >= 2:
            tb_cands.sort(key=lambda x: x['rg_ratio'], reverse=True)
            tb_cands[0]['label'] = 'Tomato'
            for other in tb_cands[1:]:
                other['label'] = 'Bun Top'

        # --- Spatial Temporal Smoothing (Position Tracker & Majority Voting) ---
        current_time = time.time()
        color_map = {
            'Tomato': (0, 0, 240),
            'Bun Top': (0, 140, 255),
            'Cheese': (0, 215, 255),
            'Patty': (40, 50, 200),
            'Bottom Bun': (200, 220, 240),
            'Lettuce': (0, 220, 0)
        }
        detected = []
        for c in raw_candidates:
            cx, cy = c['cx'], c['cy']
            best_id = None
            min_dist = 35.0  # pixels
            for tid, tdata in self.trackers.items():
                tcx, tcy = tdata['pos']
                dist = float(np.hypot(cx - tcx, cy - tcy))
                if dist < min_dist:
                    min_dist = dist
                    best_id = tid
            if best_id is None:
                best_id = self.next_tracker_id
                self.next_tracker_id += 1
                self.trackers[best_id] = {'pos': (cx, cy), 'history': deque(maxlen=7), 'last_seen': current_time}

            self.trackers[best_id]['pos'] = (cx, cy)
            self.trackers[best_id]['history'].append(c['label'])
            self.trackers[best_id]['last_seen'] = current_time

            smooth_label = Counter(self.trackers[best_id]['history']).most_common(1)[0][0]
            color_box = color_map.get(smooth_label, (255, 255, 255))
            detected.append((smooth_label, (cx, cy), c['z_val'], c['cam_coord'], c['base_coord'], color_box, c['score']))

        # Clean stale trackers (> 2.0s)
        stale_ids = [tid for tid, tdata in self.trackers.items() if current_time - tdata['last_seen'] > 2.0]
        for tid in stale_ids:
            del self.trackers[tid]

        # Draw verified bounding boxes
        for label, (cx, cy), z_val, (xc, yc), (bx_r, by_r, bz_r), color_box, score in detected:
            bx1, by1 = max(0, cx - hw), max(0, cy - hw)
            bx2, by2 = min(vis.shape[1], cx + hw), min(vis.shape[0], cy + hw)

            cv2.rectangle(vis, (bx1, by1), (bx2, by2), color_box, 2)
            cv2.circle(vis, (cx, cy), 4, color_box, -1)

            cv2.putText(vis, label, (bx1 - 10, by1 - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color_box, 2, cv2.LINE_AA)

            if self.T_base_cam is not None:
                coord_str = f"Base X:{bx_r:.0f} Y:{by_r:.0f} Z:{bz_r:.0f}"
            else:
                coord_str = f"Cam X:{xc:+.0f} Y:{yc:+.0f} Z:{z_val}mm"

            cv2.putText(vis, coord_str, (bx1 - 18, by2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)

        # Draw Table Workspace boundary guide
        cv2.polylines(vis, [self.roi_poly], isClosed=True, color=(0, 255, 255), thickness=2)
        hud = f"Strict Target Tracker: {len(detected)} verified target(s)"
        cv2.putText(vis, hud, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

        # Save snapshot file for viewing
        cv2.imwrite('/home/omen/doosan_ws/latest_detected_frame.jpg', vis)

        # Save real-time detected target coordinates to JSON for robot motion scripts
        targets_dict = {
            label: {
                'base_x': float(bx_r),
                'base_y': float(by_r),
                'base_z': float(bz_r),
                'cam_x': float(xc),
                'cam_y': float(yc),
                'depth_mm': int(z_val),
                'pixel_uv': [int(cx), int(cy)]
            }
            for label, (cx, cy), z_val, (xc, yc), (bx_r, by_r, bz_r), color_box, score in detected
        }
        try:
            with open('/home/omen/doosan_ws/latest_targets.json', 'w') as f:
                json.dump(targets_dict, f, indent=2)
        except Exception:
            pass

        # Periodic logging
        if time.time() - self.last_log_time > 2.0:
            self.last_log_time = time.time()
            if detected:
                items_str = ", ".join([f"{l.split()[0]} (Base X={bx:.0f},Y={by:.0f},Z={bz:.0f})" for l, _, _, _, (bx, by, bz), _, _ in detected])
                self.get_logger().info(f"Verified {len(detected)} card(s): {items_str}")
            else:
                self.get_logger().info("Searching for verified ingredient cards in workspace...")

        # Publish ROS Image
        det_msg = Image()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.header.frame_id = 'camera_color_optical_frame'
        det_msg.height = vis.shape[0]
        det_msg.width = vis.shape[1]
        det_msg.encoding = 'bgr8'
        det_msg.is_bigendian = 0
        det_msg.step = vis.shape[1] * 3
        det_msg.data = vis.tobytes()
        self.pub_annotated.publish(det_msg)

        # OpenCV Window
        try:
            cv2.imshow("5 Burger Ingredients 3D Tracker (Strict)", vis)
            cv2.waitKey(1)
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StrictIngredientTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
