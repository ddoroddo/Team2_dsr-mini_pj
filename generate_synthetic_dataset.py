#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate Synthetic + Real Augmented Dataset for YOLOv8 Burger Ingredients Training
- Supports 80x80mm box scale (~32-46 px)
- Includes actual camera crops (real_bun_top, live_crop_buntop, etc.)
- Emphasizes Bun Top vs Tomato differentiation
- Generates train and val sets with rich photometric and geometric augmentations
"""

import os
import random
import cv2
import numpy as np
from pathlib import Path

# Paths
WORKSPACE = Path("/home/omen/doosan_ws")
TEXTURES_DIR = WORKSPACE / "src/dsr_project/materials/textures"
DATASET_DIR = WORKSPACE / "yolo_dataset"
LIVE_FRAME_PATH = WORKSPACE / "live_camera_frame.jpg"
CLEAN_BG_PATH = WORKSPACE / "clean_table_bg.jpg"

# Classes and texture mappings
CLASS_MAP = {
    0: ("Patty", ["02_patty.jpg", "patty.jpg", "patty.png", "real_patty.jpg", "live_crop_patty.jpg"]),
    1: ("Tomato", ["tomato.jpg", "tomato.png", "real_tomato.jpg", "live_crop_tomato.jpg"]),
    2: ("Bottom Bun", ["bottom_bun.jpg", "bottom_bun.png", "real_bottom_bun.jpg", "live_crop_bottombun.jpg"]),
    3: ("Bun Top", ["01_bun_top.jpg", "upper_bun.jpg", "upper_bun.png", "real_bun_top.jpg", "live_crop_buntop.jpg"]),
    4: ("Cheese", ["04_cheese.jpg", "cheese.jpg", "cheese.png", "real_cheese.jpg", "live_crop_cheese.jpg"]),
}

# Table workspace polygon where cards sit on desk
ROI_POLY = np.array([
    [230, 100],   # Top-Left
    [460, 100],   # Top-Right
    [485, 430],   # Bottom-Right
    [270, 430]    # Bottom-Left
], dtype=np.int32)

# Ground truth boxes on live_camera_frame.jpg (format: class_id, x1, y1, x2, y2)
REAL_LABELS = [
    (2, 237, 214, 285, 264),  # Bottom Bun
    (3, 294, 224, 341, 269),  # Bun Top
    (4, 358, 219, 401, 263),  # Cheese
    (1, 258, 265, 315, 322),  # Tomato
    (0, 333, 269, 380, 317),  # Patty
]


def is_inside_roi(cx, cy):
    return cv2.pointPolygonTest(ROI_POLY, (float(cx), float(cy)), False) >= 0


def load_card_textures():
    textures = {}
    for cls_id, (cls_name, file_names) in CLASS_MAP.items():
        imgs = []
        for fn in file_names:
            fp = TEXTURES_DIR / fn
            if fp.exists():
                img = cv2.imread(str(fp))
                if img is not None and img.size > 0:
                    is_real_crop = ("real_" in fn or "live_crop_" in fn)
                    imgs.append((img, is_real_crop, fn))
        if not imgs:
            raise RuntimeError(f"No texture images found for {cls_name}!")
        textures[cls_id] = imgs
        print(f"Loaded {len(imgs)} texture/crop files for {cls_name}")
    return textures


def load_background_templates():
    bgs = []
    if CLEAN_BG_PATH.exists():
        bg = cv2.imread(str(CLEAN_BG_PATH))
        if bg is not None:
            bgs.append(bg)
    if LIVE_FRAME_PATH.exists():
        # Mask out existing cards on live frame with local inpainting / blur to create extra background
        live = cv2.imread(str(LIVE_FRAME_PATH))
        if live is not None:
            mask = np.zeros(live.shape[:2], dtype=np.uint8)
            for _, x1, y1, x2, y2 in REAL_LABELS:
                cv2.rectangle(mask, (x1 - 5, y1 - 5), (x2 + 5, y2 + 5), 255, -1)
            inpainted = cv2.inpaint(live, mask, 7, cv2.INPAINT_TELEA)
            bgs.append(inpainted)
    if not bgs:
        blank = np.full((480, 640, 3), (70, 75, 80), dtype=np.uint8)
        bgs.append(blank)
    return bgs


def make_card_box(img_info, target_size=38):
    """
    Creates realistic 80x80mm card appearance:
    - target_size: ~34-44 pixels
    """
    tex_img, is_real_crop, _ = img_info

    if is_real_crop:
        # Real crop already has the card texture & margins
        card = cv2.resize(tex_img, (target_size, target_size), interpolation=cv2.INTER_AREA)
    else:
        # Synthetic card: inner texture with white margin (15-20%)
        inner_size = max(16, int(target_size * 0.76))
        inner = cv2.resize(tex_img, (inner_size, inner_size), interpolation=cv2.INTER_AREA)

        # Realistic off-white paper tone
        paper_val = random.randint(230, 252)
        card = np.full((target_size, target_size, 3), paper_val, dtype=np.uint8)
        offset = (target_size - inner_size) // 2
        card[offset:offset + inner_size, offset:offset + inner_size] = inner

        # Subtle card border
        cv2.rectangle(card, (0, 0), (target_size - 1, target_size - 1), (190, 190, 190), 1)

    return card


def rotate_card(card_img, angle):
    """Rotates card and generates transparent/mask background"""
    h, w = card_img.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    M[0, 2] += (nW / 2) - center[0]
    M[1, 2] += (nH / 2) - center[1]

    b, g, r = cv2.split(card_img)
    alpha = np.full((h, w), 255, dtype=np.uint8)
    bgra = cv2.merge([b, g, r, alpha])

    rotated = cv2.warpAffine(bgra, M, (nW, nH), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
    return rotated, nW, nH


def overlay_card(bg_img, rotated_card, cx, cy):
    rh, rw = rotated_card.shape[:2]
    x1 = cx - rw // 2
    y1 = cy - rh // 2
    x2 = x1 + rw
    y2 = y1 + rh

    img_h, img_w = bg_img.shape[:2]
    if x1 < 0 or y1 < 0 or x2 >= img_w or y2 >= img_h:
        return False, None

    card_bgr = rotated_card[:, :, :3]
    card_alpha = rotated_card[:, :, 3] / 255.0

    roi = bg_img[y1:y2, x1:x2]
    for c in range(3):
        roi[:, :, c] = (card_alpha * card_bgr[:, :, c] + (1.0 - card_alpha) * roi[:, :, c]).astype(np.uint8)

    bg_img[y1:y2, x1:x2] = roi
    norm_cx = cx / img_w
    norm_cy = cy / img_h
    norm_w = rw / img_w
    norm_h = rh / img_h
    return True, (norm_cx, norm_cy, norm_w, norm_h)


def apply_lighting_jitter(img):
    """Realistic lighting, contrast, and noise variations"""
    alpha = random.uniform(0.85, 1.15)
    beta = random.randint(-15, 15)
    jittered = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    # Slight color shift in HSV
    if random.random() < 0.4:
        hsv = cv2.cvtColor(jittered, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 0] = (hsv[:, :, 0] + random.uniform(-4, 4)) % 180
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * random.uniform(0.9, 1.1), 0, 255)
        jittered = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    # Blur
    if random.random() < 0.2:
        k = random.choice([3, 5])
        jittered = cv2.GaussianBlur(jittered, (k, k), 0)
    return jittered


def generate_augmented_real_samples(split_name, count, img_dir, lbl_dir, start_idx=0):
    """Augment the real camera frame directly to ensure 100% domain accuracy"""
    if not LIVE_FRAME_PATH.exists():
        return 0

    base_frame = cv2.imread(str(LIVE_FRAME_PATH))
    if base_frame is None:
        return 0

    h, w = base_frame.shape[:2]
    print(f"Adding {count} real-image augmented samples to {split_name}...")

    for i in range(count):
        aug_img = base_frame.copy()
        alpha = random.uniform(0.88, 1.12)
        beta = random.randint(-12, 12)
        aug_img = cv2.convertScaleAbs(aug_img, alpha=alpha, beta=beta)

        if random.random() < 0.3:
            hsv = cv2.cvtColor(aug_img, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[:, :, 0] = (hsv[:, :, 0] + random.uniform(-3, 3)) % 180
            aug_img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        labels = []
        for cls_id, x1, y1, x2, y2 in REAL_LABELS:
            # Slight random box jitter (+/- 1-2 px)
            jx1 = x1 + random.randint(-1, 1)
            jy1 = y1 + random.randint(-1, 1)
            jx2 = x2 + random.randint(-1, 1)
            jy2 = y2 + random.randint(-1, 1)
            cx = (jx1 + jx2) / 2.0 / w
            cy = (jy1 + jy2) / 2.0 / h
            bw = (jx2 - jx1) / w
            bh = (jy2 - jy1) / h
            labels.append((cls_id, cx, cy, bw, bh))

        file_id = f"real_{split_name}_{start_idx + i:04d}"
        cv2.imwrite(str(img_dir / f"{file_id}.jpg"), aug_img)
        with open(lbl_dir / f"{file_id}.txt", "w") as f:
            for l in labels:
                f.write(f"{l[0]} {l[1]:.6f} {l[2]:.6f} {l[3]:.6f} {l[4]:.6f}\n")

    return count


def generate_samples(split_name, num_samples, textures, bgs, real_aug_count=20):
    img_dir = DATASET_DIR / "images" / split_name
    lbl_dir = DATASET_DIR / "labels" / split_name
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating {num_samples} synthetic samples for '{split_name}' split...")

    for i in range(num_samples):
        bg = random.choice(bgs).copy()
        # Ensure all 5 classes appear frequently, especially Bun Top (id=3) and Tomato (id=1)
        num_cards = random.randint(3, 5)
        # 70% chance to force both Bun Top and Tomato in the same scene to learn contrast
        if random.random() < 0.70:
            mandatory = [1, 3]
            others = [c for c in CLASS_MAP.keys() if c not in mandatory]
            chosen_classes = mandatory + random.sample(others, num_cards - 2)
        else:
            chosen_classes = random.sample(list(CLASS_MAP.keys()), num_cards)

        labels = []
        placed_centers = []

        for cls_id in chosen_classes:
            tex_info = random.choice(textures[cls_id])

            # 80x80 mm box scale corresponds to ~34-44 px at ~0.95m table height
            card_sz = random.randint(34, 46)
            card = make_card_box(tex_info, target_size=card_sz)

            angle = random.randint(0, 360)
            rot_card, rw, rh = rotate_card(card, angle)

            for _ in range(60):
                cx = random.randint(235 + rw // 2, 475 - rw // 2)
                cy = random.randint(105 + rh // 2, 425 - rh // 2)
                if not is_inside_roi(cx, cy):
                    continue
                if all(np.hypot(cx - px, cy - py) > 48 for px, py in placed_centers):
                    ok, bbox = overlay_card(bg, rot_card, cx, cy)
                    if ok:
                        placed_centers.append((cx, cy))
                        labels.append((cls_id, *bbox))
                    break

        final_img = apply_lighting_jitter(bg)
        file_id = f"synth_{split_name}_{i:04d}"
        cv2.imwrite(str(img_dir / f"{file_id}.jpg"), final_img)
        with open(lbl_dir / f"{file_id}.txt", "w") as f:
            for l in labels:
                f.write(f"{l[0]} {l[1]:.6f} {l[2]:.6f} {l[3]:.6f} {l[4]:.6f}\n")

    # Add real augmented samples
    generate_augmented_real_samples(split_name, real_aug_count, img_dir, lbl_dir)
    print(f"✅ Finished generating samples for {split_name}.")


def main():
    print("Loading textures and backgrounds...")
    textures = load_card_textures()
    bgs = load_background_templates()

    print(f"Loaded {len(bgs)} background templates and 5 card texture sets.")

    # Clean old dataset images/labels
    for split in ["train", "val"]:
        for sub in ["images", "labels"]:
            p = DATASET_DIR / sub / split
            if p.exists():
                for f in p.glob("*"):
                    f.unlink()

    # Generate 400 Train images (360 synth + 40 real aug) and 80 Val images (70 synth + 10 real aug)
    generate_samples("train", 360, textures, bgs, real_aug_count=40)
    generate_samples("val", 70, textures, bgs, real_aug_count=10)

    print("\n🎉 Dataset Generation Complete!")
    print(f"   Location: {DATASET_DIR}")


if __name__ == "__main__":
    main()
