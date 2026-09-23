#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Train Lightweight YOLOv8n on Burger Ingredients Dataset using RTX 5080 GPU
"""

from ultralytics import YOLO
from pathlib import Path

DATA_YAML = "/home/omen/doosan_ws/yolo_dataset/burger.yaml"
PROJECT_DIR = "/home/omen/doosan_ws/yolo_runs"

def main():
    print("🚀 Initializing YOLOv8n model...")
    model = YOLO("yolov8n.pt")

    print("🔥 Starting training on NVIDIA RTX 5080 (device=0)...")
    results = model.train(
        data=DATA_YAML,
        epochs=45,
        imgsz=640,
        batch=16,
        device=0,
        project=PROJECT_DIR,
        name="burger_yolov8n",
        exist_ok=True,
        workers=4,
        patience=15,
        save=True,
        verbose=True
    )

    best_model_path = Path(results.save_dir) / "weights" / "best.pt"
    print(f"\n🎉 Training complete!")
    print(f"   Best Model weights: {best_model_path}")

if __name__ == "__main__":
    main()
