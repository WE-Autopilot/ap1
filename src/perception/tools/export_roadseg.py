"""Export the road segmentation model to ONNX and check it gives the same answer.

Ultralytics is only needed here, not at runtime, so it isn't a project dependency.
Usage (from src/perception):
    uv run --with "ultralytics>=8.4.52" python tools/export_roadseg.py [road_photo.jpg]

What it does:
    1. Downloads the pretrained yolo26n-sem.pt (Cityscapes, 19 classes) into
       ~/.cache/ap1, so no weights end up in the repo.
    2. Exports it to models/yolo26n-sem.onnx at the input size from #91
       (768 wide x 448 high, fixed). The export bakes in ArgMax, so the model
       outputs a uint8 class map directly. Commit the result with Git LFS.
    3. With a photo: runs the .pt through ultralytics and the .onnx through
       ap1_perception.road_seg.RoadSeg (the node's own preprocessing), prints how
       many pixels agree (should be ~99%) and the time per image, and saves
       <photo>_onnx_road.jpg with road green and sidewalk blue.
"""

import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

PERCEPTION_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PERCEPTION_DIR))
from ap1_perception.road_seg import RoadSeg  # noqa: E402

WEIGHTS = Path.home() / ".cache" / "ap1" / "yolo26n-sem.pt"
OUTPUT = PERCEPTION_DIR / "models" / "yolo26n-sem.onnx"
IMGSZ = (448, 768)  # (height, width): 1920x1080 -> 768x432, padded to 768x448
ROAD = 0
SIDEWALK = 1


def export() -> Path:
    WEIGHTS.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(WEIGHTS))  # downloads to WEIGHTS the first time
    exported = model.export(format="onnx", imgsz=IMGSZ, opset=17, simplify=True)
    shutil.move(exported, OUTPUT)
    print(f"\nExported: {OUTPUT}")
    return OUTPUT


def compare(onnx_path: Path, image_path: Path) -> None:
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        sys.exit(f"Could not read image: {image_path}")

    # Reference: ultralytics on the .pt, at the same input size, CPU
    pt = YOLO(str(WEIGHTS))
    pt(bgr, imgsz=IMGSZ, device="cpu", verbose=False)  # warm-up (first run is always slow)
    start = time.perf_counter()
    pt_map = pt(bgr, imgsz=IMGSZ, device="cpu", verbose=False)[0].semantic_mask.data.cpu().numpy()
    pt_ms = (time.perf_counter() - start) * 1000

    # What the node runs: RoadSeg on the .onnx, CPU
    seg = RoadSeg(onnx_path, device="cpu")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    seg(rgb)
    start = time.perf_counter()
    onnx_map = seg(rgb)
    onnx_ms = (time.perf_counter() - start) * 1000

    # RoadSeg returns the model resolution; compare at that size
    h, w = onnx_map.shape
    pt_map = cv2.resize(pt_map.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
    print(f"Class map size: {w}x{h}")
    print(f"Pixels where .pt and .onnx agree: {100 * np.mean(pt_map == onnx_map):.2f}%")
    print(f"Road pixels  .pt: {100 * np.mean(pt_map == ROAD):.1f}%   .onnx: {100 * np.mean(onnx_map == ROAD):.1f}%")
    print(f"Time per image (CPU)  .pt: {pt_ms:.0f} ms   .onnx: {onnx_ms:.0f} ms")

    small = cv2.resize(bgr, (w, h))
    overlay = small.copy()
    overlay[onnx_map == ROAD] = (0, 255, 0)
    overlay[onnx_map == SIDEWALK] = (255, 0, 0)
    out = image_path.with_name(image_path.stem + "_onnx_road.jpg")
    cv2.imwrite(str(out), cv2.addWeighted(small, 0.5, overlay, 0.5, 0))
    print(f"Saved: {out}")


def main():
    if len(sys.argv) > 2:
        print(__doc__)
        sys.exit(1)
    onnx_path = export()
    if len(sys.argv) == 2:
        compare(onnx_path, Path(sys.argv[1]).resolve())


if __name__ == "__main__":
    main()
