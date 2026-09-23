#!/usr/bin/env python3
"""버거 재료 원본 사진(흰 배경, 알파 없음)을 웹용 투명 PNG로 변환한다.

사용법:
  python3 scripts/prepare_assets.py --src "/home/lsh/Downloads/burger images" \
      --out assets/ingredients --width 320 [--pad 8] [--keep-shadow] [--preview]

알고리즘:
  1. d = 255 - min(R,G,B), sat = (max-min)/max 로 "흰색/회색 그림자" 후보 마스크 생성
  2. 이미지 테두리에 닿은 연결 성분만 배경으로 취급 (빵 속살처럼 물체 내부의 흰 영역 보존)
  3. 2px 침식으로 흰 안티에일리어싱 테두리 제거
  4. bbox + pad 크롭, 하단 번 물체 폭 기준 균일 스케일로 리사이즈(LANCZOS)
  5. 결과 검증(코너 alpha 0, 중앙 alpha 255) 후 표 출력
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

# 원본 파일명 -> 출력 id (스택 순서대로)
FILE_MAP = {
    "bottom_bun.png": "bottom_bun",
    "patty.png": "patty",
    "cheese.png": "cheese",
    "tomato.png": "tomato",
    "upper_bun.png": "top_bun",
}
REFERENCE_ID = "bottom_bun"  # 이 재료의 물체 폭이 --width 가 되도록 균일 스케일


def object_mask(rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """물체 마스크(bool)와 배경 밝기 거리 d(0~255)를 반환."""
    a = rgb.astype(np.int32)
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    d = 255 - mn
    sat = (mx - mn) / np.maximum(mx, 1)
    bg_candidate = (d <= 40) | ((sat < 0.10) & (d <= 120))

    lab, n = ndimage.label(bg_candidate)
    border = np.concatenate([lab[0, :], lab[-1, :], lab[:, 0], lab[:, -1]])
    border_labels = np.unique(border[border > 0])
    background = np.isin(lab, border_labels)
    obj = ~background
    obj = ndimage.binary_fill_holes(obj)
    obj = ndimage.binary_erosion(obj, iterations=2, border_value=0)
    return obj, d


def process(src: Path, out_id: str, keep_shadow: bool, pad: int):
    img = Image.open(src).convert("RGB")
    rgb = np.asarray(img)
    obj, d = object_mask(rgb)

    alpha = np.where(obj, 255, 0).astype(np.uint8)
    if keep_shadow:
        shadow = np.clip((d - 4) / (120 - 4), 0, 1) * 0.55 * 255
        alpha = np.where(obj, 255, shadow).astype(np.uint8)

    ys, xs = np.where(obj)
    if len(xs) == 0:
        raise SystemExit(f"[{out_id}] 물체를 찾지 못했습니다: {src}")
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, rgb.shape[1])
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, rgb.shape[0])

    rgba = np.dstack([rgb, alpha])[y0:y1, x0:x1]
    obj_w = int(xs.max() - xs.min() + 1)
    obj_h = int(ys.max() - ys.min() + 1)
    return Image.fromarray(rgba, "RGBA"), obj_w, obj_h


def verify(path: Path):
    im = Image.open(path)
    assert im.mode == "RGBA", f"{path.name}: mode {im.mode}"
    w, h = im.size
    a = np.asarray(im)[:, :, 3]
    corners = [a[0, 0], a[0, -1], a[-1, 0], a[-1, -1]]
    assert all(c == 0 for c in corners), f"{path.name}: 코너 alpha != 0 ({corners})"
    assert a[h // 2, w // 2] == 255, f"{path.name}: 중앙 alpha != 255"
    semi = int(((a > 0) & (a < 255)).sum())
    return semi


def write_preview(out_dir: Path, ids: list[str], widths: dict[str, int]):
    """thickness 값을 써서 스택 합성 프리뷰를 만든다(브라우저 없이 빠른 확인용)."""
    thickness = {"bottom_bun": 0.16, "patty": 0.085, "cheese": 0.03, "tomato": 0.075, "top_bun": 0.755}
    order = ["bottom_bun", "patty", "cheese", "tomato", "top_bun"]
    W = widths[REFERENCE_ID]
    layers = [Image.open(out_dir / f"{i}.png") for i in order]
    total_h = int(W * (sum(thickness[i] for i in order[:-1]) + layers[-1].height / W)) + 40
    canvas = Image.new("RGBA", (int(W * 1.4), total_h), (42, 157, 143, 255))
    y_base = canvas.height - 20
    lift = 0.0
    for i, im in zip(order, layers):
        x = (canvas.width - im.width) // 2
        y = int(y_base - W * lift - im.height)
        canvas.alpha_composite(im, (x, y))
        lift += thickness[i]
    p = out_dir / "_preview_stack.png"
    canvas.save(p)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True, help="원본 PNG 폴더")
    ap.add_argument("--out", default="assets/ingredients", help="출력 폴더")
    ap.add_argument("--width", type=int, default=320, help="하단 번 물체 폭(px) 기준 크기")
    ap.add_argument("--pad", type=int, default=8, help="bbox 여백(px, 원본 기준)")
    ap.add_argument("--keep-shadow", action="store_true", help="접촉 그림자를 반투명으로 유지")
    ap.add_argument("--preview", action="store_true", help="_preview_stack.png 합성 출력")
    args = ap.parse_args()

    src_dir, out_dir = Path(args.src), Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    results = {}
    for fname, out_id in FILE_MAP.items():
        p = src_dir / fname
        if not p.exists():
            raise SystemExit(f"원본이 없습니다: {p}")
        results[out_id] = process(p, out_id, args.keep_shadow, args.pad)

    ref_w = results[REFERENCE_ID][1]
    factor = args.width / ref_w

    print(f"{'id':<12}{'obj(px)':>12}{'out(px)':>12}{'scale':>8}{'h/w':>8}{'KB':>8}{'semi':>8}")
    widths = {}
    for out_id, (im, obj_w, obj_h) in results.items():
        new_size = (max(1, round(im.width * factor)), max(1, round(im.height * factor)))
        small = im.resize(new_size, Image.LANCZOS)
        out_path = out_dir / f"{out_id}.png"
        small.save(out_path, optimize=True)
        semi = verify(out_path)
        scale = obj_w / ref_w
        widths[out_id] = round(obj_w * factor)
        print(f"{out_id:<12}{f'{obj_w}x{obj_h}':>12}{f'{small.width}x{small.height}':>12}"
              f"{scale:>8.3f}{obj_h / obj_w:>8.3f}{out_path.stat().st_size / 1024:>8.1f}{semi:>8}")

    if args.preview:
        print("preview:", write_preview(out_dir, list(FILE_MAP.values()), widths))
    print("완료:", out_dir.resolve())


if __name__ == "__main__":
    sys.exit(main())
