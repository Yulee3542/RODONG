#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rodong_train.py  (RODONG12 - YOLO training pipeline, from scratch)
================================================================
Dataset: Roboflow "Stairs & ramps" (Vasile Grosu)
  - ramp  -> CLIMB (class 0)
  - stairs-> AVOID (class 1)
Environment: mini PC i5-6500T (CPU only, no GPU)
Goal: train YOLOv8n 2-class -> ONNX export -> deploy to Pi

CPU training optimization principles:
  - input 320 (~4x faster than 640)
  - fewer epochs (50), small batch (8)
  - fewer workers (4 cores), light augmentation
  - nano model (yolov8n)

Usage order:
  1) pip install ultralytics roboflow
  2) Get an API key from Roboflow (free): https://app.roboflow.com -> Settings -> API
  3) Set the ROBOFLOW_API_KEY env var, or enter it directly below
  4) python3 rodong_train.py --download   # download dataset + remap classes
  5) python3 rodong_train.py --train      # CPU training
  6) python3 rodong_train.py --export     # ONNX export (for the Pi)
  7) python3 rodong_train.py --all        # run all of the above in sequence
"""

import os
import sys
import argparse
import shutil
import glob

# ── Settings ─────────────────────────────────────────────────────
ROBOFLOW_WORKSPACE = "vasile-grosu-uslqx"
ROBOFLOW_PROJECT   = "stairs-ramps"
ROBOFLOW_VERSION   = 1            # on first run, check the latest version on the site and adjust

# Output paths
WORK_DIR   = os.path.expanduser("~/2. OWOD for Rodong/rodong_yolo")
DATA_DIR   = os.path.join(WORK_DIR, "dataset")
RUNS_DIR   = os.path.join(WORK_DIR, "runs")
ONNX_OUT   = os.path.join(WORK_DIR, "rodong.onnx")

# Class mapping: Roboflow original class name -> RODONG class ID
# (after download, check the names in data.yaml and adjust to the exact original names)
CLASS_MAP = {
    "ramp":   0,   # CLIMB
    "ramps":  0,
    "stair":  1,   # AVOID
    "stairs": 1,
}
RODONG_NAMES = ["CLIMB", "AVOID"]   # final 2-class

# Training hyperparameters (CPU only)
TRAIN_CFG = dict(
    model    = "yolov8n.pt",   # nano (lightest)
    imgsz    = 320,            # small for CPU speedup
    epochs   = 50,
    batch    = 8,
    workers  = 4,              # i5 4 cores
    patience = 15,             # early stopping
    device   = "cpu",
    optimizer= "auto",
    # Light augmentation (lower CPU load)
    mosaic   = 0.5,
    mixup    = 0.0,
    hsv_h    = 0.015,
    hsv_s    = 0.5,
    hsv_v    = 0.4,
)


# ────────────────────────────────────────────────────────────────
def step_download():
    """Download the Roboflow dataset + remap to RODONG 2-class."""
    try:
        from roboflow import Roboflow
    except ImportError:
        sys.exit("ERROR: run 'pip install roboflow' first.")

    api_key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
    if not api_key:
        sys.exit("ERROR: set the ROBOFLOW_API_KEY env var.\n"
                 "  export ROBOFLOW_API_KEY='your_key_here'")

    os.makedirs(WORK_DIR, exist_ok=True)
    print(f"[download] downloading {ROBOFLOW_PROJECT} v{ROBOFLOW_VERSION} from Roboflow...")

    rf = Roboflow(api_key=api_key)
    proj = rf.workspace(ROBOFLOW_WORKSPACE).project(ROBOFLOW_PROJECT)
    dataset = proj.version(ROBOFLOW_VERSION).download("yolov8", location=DATA_DIR)

    print(f"[download] done: {DATA_DIR}")
    print("[download] check the names in data.yaml and align CLASS_MAP:")
    _print_data_yaml()
    _remap_labels()


def _print_data_yaml():
    yml = os.path.join(DATA_DIR, "data.yaml")
    if os.path.exists(yml):
        print("──── data.yaml ────")
        with open(yml) as f:
            print(f.read())
        print("───────────────────")


def _remap_labels():
    """Rewrite label files from original class ID -> RODONG ID.
    Reads the names order from the original data.yaml and converts using CLASS_MAP."""
    import yaml
    yml = os.path.join(DATA_DIR, "data.yaml")
    with open(yml) as f:
        cfg = yaml.safe_load(f)
    orig_names = cfg.get("names", [])
    print(f"[remap] original classes: {orig_names}")

    # original ID -> RODONG ID conversion table
    id_map = {}
    for orig_id, name in enumerate(orig_names):
        key = str(name).strip().lower()
        if key in CLASS_MAP:
            id_map[orig_id] = CLASS_MAP[key]
        else:
            id_map[orig_id] = None   # class to drop
    print(f"[remap] ID conversion table (original->RODONG, None=removed): {id_map}")

    # rewrite label .txt files for all splits
    for split in ("train", "valid", "test"):
        lbl_dir = os.path.join(DATA_DIR, split, "labels")
        if not os.path.isdir(lbl_dir):
            continue
        n_files, n_boxes, n_dropped = 0, 0, 0
        for txt in glob.glob(os.path.join(lbl_dir, "*.txt")):
            new_lines = []
            with open(txt) as f:
                for line in f:
                    parts = line.split()
                    if not parts:
                        continue
                    oid = int(parts[0])
                    nid = id_map.get(oid)
                    if nid is None:
                        n_dropped += 1
                        continue
                    new_lines.append(" ".join([str(nid)] + parts[1:]))
                    n_boxes += 1
            with open(txt, "w") as f:
                f.write("\n".join(new_lines) + ("\n" if new_lines else ""))
            n_files += 1
        print(f"[remap] {split}: {n_files} files, {n_boxes} boxes kept, {n_dropped} boxes removed")

    # overwrite data.yaml names with RODONG 2-class
    cfg["names"] = RODONG_NAMES
    cfg["nc"]    = len(RODONG_NAMES)
    with open(yml, "w") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True)
    print(f"[remap] data.yaml updated: nc={len(RODONG_NAMES)}, names={RODONG_NAMES}")


def step_train():
    """CPU training."""
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ERROR: run 'pip install ultralytics' first.")

    yml = os.path.join(DATA_DIR, "data.yaml")
    if not os.path.exists(yml):
        sys.exit(f"ERROR: {yml} not found. Run --download first.")

    print("[train] starting CPU training (takes a long time)...")
    model = YOLO(TRAIN_CFG["model"])
    model.train(
        data     = yml,
        imgsz    = TRAIN_CFG["imgsz"],
        epochs   = TRAIN_CFG["epochs"],
        batch    = TRAIN_CFG["batch"],
        workers  = TRAIN_CFG["workers"],
        patience = TRAIN_CFG["patience"],
        device   = TRAIN_CFG["device"],
        optimizer= TRAIN_CFG["optimizer"],
        mosaic   = TRAIN_CFG["mosaic"],
        mixup    = TRAIN_CFG["mixup"],
        hsv_h    = TRAIN_CFG["hsv_h"],
        hsv_s    = TRAIN_CFG["hsv_s"],
        hsv_v    = TRAIN_CFG["hsv_v"],
        project  = RUNS_DIR,
        name     = "rodong_yolov8n",
        exist_ok = True,
    )
    print(f"[train] done. results: {RUNS_DIR}/rodong_yolov8n/")


def step_export():
    """best.pt -> ONNX export (Pi OpenCV DNN compatible)."""
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ERROR: run 'pip install ultralytics' first.")

    best = os.path.join(RUNS_DIR, "rodong_yolov8n", "weights", "best.pt")
    if not os.path.exists(best):
        sys.exit(f"ERROR: {best} not found. Run --train first.")

    print(f"[export] {best} -> ONNX...")
    model = YOLO(best)
    # opset 12 = good OpenCV DNN compatibility. simplify reduces the graph.
    out = model.export(
        format   = "onnx",
        imgsz    = TRAIN_CFG["imgsz"],
        opset    = 12,
        simplify = True,
        dynamic  = False,   # fixed input is safe for Pi OpenCV DNN
    )
    # copy the export result to ONNX_OUT
    if isinstance(out, str) and os.path.exists(out):
        shutil.copy(out, ONNX_OUT)
    else:
        src = best.replace(".pt", ".onnx")
        if os.path.exists(src):
            shutil.copy(src, ONNX_OUT)
    print(f"[export] done: {ONNX_OUT}")
    print(f"[export] copy to the Pi:")
    print(f"  scp '{ONNX_OUT}' pi@192.168.10.2:~/xycar_ws/src/rodong/models/rodong.onnx")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--download", action="store_true", help="download dataset + remap")
    ap.add_argument("--train",    action="store_true", help="CPU training")
    ap.add_argument("--export",   action="store_true", help="ONNX export")
    ap.add_argument("--all",      action="store_true", help="run all in sequence")
    args = ap.parse_args()

    if args.all:
        step_download(); step_train(); step_export()
    elif args.download:
        step_download()
    elif args.train:
        step_train()
    elif args.export:
        step_export()
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
