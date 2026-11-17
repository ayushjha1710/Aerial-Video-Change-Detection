# Drone Change Detection System

Detects and logs changes between two drone videos of the same location,
using GPS coordinates to align frames and YOLOv8 for object-level analysis.

---

## Project Structure

```
drone_change_detection/
├── main.py                  # CLI entry point
├── change_detector.py       # Core pipeline & data structures
├── requirements.txt
└── utils/
    ├── gps_matcher.py       # GPS-based frame pairing (Haversine + KD-tree)
    ├── frame_aligner.py     # ECC + ORB homography alignment
    ├── change_analyzer.py   # Pixel-diff + morphology change scoring
    └── report_generator.py  # Standalone HTML report
```

---

## Installation

```bash
pip install -r requirements.txt
```

For GPU support (recommended), install the correct PyTorch version for your CUDA:
```bash
# Example for CUDA 11.8
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

---

## GPS CSV Format

One CSV per video, with **no spaces** around values:

```
frame_index,latitude,longitude
0,26.7271,88.3953
1,26.7272,88.3953
2,26.7273,88.3954
...
```

---

## Usage

### Basic

```bash
python main.py \
  --before-video  before.mp4 \
  --after-video   after.mp4 \
  --before-gps    before_gps.csv \
  --after-gps     after_gps.csv \
  --model         runs/train/exp/weights/best.pt
```

### With all options

```bash
python main.py \
  --before-video    before.mp4 \
  --after-video     after.mp4 \
  --before-gps      before_gps.csv \
  --after-gps       after_gps.csv \
  --model           best.pt \
  --output          output/ \
  --gps-tolerance   5.0 \       # metres radius for a GPS match
  --change-threshold 0.15 \     # 0-1 composite score cutoff
  --pixel-diff      30 \        # per-pixel abs diff to count as changed
  --min-change-pct  2.0 \       # ignore frames with < 2% pixel change
  --yolo-conf       0.25 \      # YOLO confidence
  --yolo-iou        0.45 \      # YOLO NMS IoU
  --ecc-iterations  1000        # alignment convergence budget
```

---

## Outputs

```
output/
├── frames/
│   ├── before/   # Saved before-frames for each change event
│   ├── after/    # Saved after-frames (aligned)
│   └── diff/     # Side-by-side diff visualisation (Before | Heatmap | After)
├── detections/
│   ├── change_events.json   # Full structured results
│   └── change_events.csv    # Spreadsheet-friendly summary
└── reports/
    └── change_report.html   # Standalone visual report (open in any browser)
```

### JSON schema (per event)

```json
{
  "gps_lat": 26.7271,
  "gps_lon": 88.3953,
  "before_frame_idx": 12,
  "after_frame_idx": 9,
  "change_score": 0.412,
  "pixel_change_pct": 18.3,
  "objects_before": [{"class": "car", "confidence": 0.87, "bbox": [...]}],
  "objects_after":  [{"class": "car", "confidence": 0.91, "bbox": [...]},
                     {"class": "truck", "confidence": 0.72, "bbox": [...]}],
  "new_objects":     [{"class": "truck", ...}],
  "removed_objects": [],
  "change_type": "object",
  "before_frame_path": "output/frames/before/lat26.72710_lon88.39530_b12_a9.jpg",
  "after_frame_path":  "output/frames/after/lat26.72710_lon88.39530_b12_a9.jpg",
  "diff_frame_path":   "output/frames/diff/lat26.72710_lon88.39530_b12_a9_diff.jpg"
}
```

---

## How It Works

### 1 — GPS Frame Matching (`gps_matcher.py`)
- Converts all GPS coordinates to a local XY metre plane (no projection library needed).
- Builds a **cKDTree** on before-video GPS points.
- Queries each after-video frame's position against the tree.
- Pairs are kept only if distance ≤ `--gps-tolerance` metres.
- Each before-frame is matched at most once (1-to-1, best-first).

### 2 — Frame Alignment (`frame_aligner.py`)
- **ECC (Enhanced Correlation Coefficient)** estimates an affine warp between grayscale frames. Robust to illumination changes and drone vibration.
- If ECC fails (low texture, fog), **ORB features + RANSAC homography** is used as a fallback.
- The warp is applied to the after-frame so both frames are pixel-registered.

### 3 — Change Analysis (`change_analyzer.py`)
- Gaussian blur suppresses sensor noise.
- Absolute pixel difference → threshold → binary change mask.
- Morphological open+close removes speckles and closes small gaps.
- **Change score** = 0.6 × (changed pixel fraction) + 0.4 × (mean diff intensity).
- A colour diff panel is saved: **Before | JET heatmap | After**, with changed contours outlined in red.

### 4 — YOLOv8 Detection (`change_detector.py`)
- YOLO runs on both the before and aligned after frame.
- Object classes are diffed: classes in after but not before → **new objects**; classes in before but not after → **removed objects**.
- `change_type` is classified as `structural`, `object`, `structural+object`, or `minor`.

### 5 — Reporting (`report_generator.py`)
- JSON + CSV with full per-event data.
- Standalone HTML report with summary table and per-event side-by-side frame panels.

---

## Tips

| Issue | Fix |
|---|---|
| Too many false positives | Increase `--change-threshold` (e.g. 0.25) or `--pixel-diff` (e.g. 50) |
| Missing real changes | Decrease `--change-threshold` (e.g. 0.08) or `--pixel-diff` (e.g. 20) |
| GPS pairs not found | Increase `--gps-tolerance` (e.g. 10–20 m) |
| Slow on large video | Use `--no-save-frames` to skip frame I/O |
| Alignment failing | Try `--no-homography` if frames are very different in angle |
