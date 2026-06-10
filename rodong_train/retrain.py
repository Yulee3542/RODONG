#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
retrain.py  (RODONG13 - retrain on existing data, AVOID-focused)
================================================================
Retrain YOLOv8n on the existing Roboflow "Stairs & ramps" dataset
(already remapped to 0=CLIMB / 1=AVOID). Improves only hyperparameters/augmentation,
without collecting new data.

Design decisions (keep deployment compatibility):
  - Keep 2-class (CLIMB/AVOID) -> no change to ONNX output shape / yolo_node.py.
    At runtime, USE_CLIMB=False, PUBLISH_ONLY_AVOID=True uses AVOID only.
  - imgsz=320, yolov8n -> keep Pi inference speed / input size unchanged.
  - Small data (275 train images) -> strong augmentation to curb overfitting and improve generalization.

Path robustness:
  - rodong_yolo moved to the repo root, so resolve paths automatically relative to the script location.
  - To avoid the '../' relative-path issue in Roboflow's data.yaml, generate an absolute-path data_local.yaml.

Usage:
  python3 retrain.py --all                 # train -> ONNX export -> test eval
  python3 retrain.py --train [--epochs N]  # train only
  python3 retrain.py --export              # best.pt -> ONNX
  python3 retrain.py --eval                # evaluate test split
  python3 retrain.py --smoke               # 1-epoch pipeline check (temp folder)
"""

import os
import sys
import shutil
import argparse

# ── Paths (resolved automatically relative to the script location) ──
HERE       = os.path.dirname(os.path.abspath(__file__))     # .../RODONG/rodong_train
REPO       = os.path.dirname(HERE)                           # .../RODONG
YOLO_DIR   = os.path.join(REPO, "rodong_yolo")
DATA_DIR   = os.path.join(YOLO_DIR, "dataset")
RUNS_DIR   = os.path.join(YOLO_DIR, "runs")
BASE_MODEL = os.path.join(HERE, "yolov8n.pt")               # pretrained weights
ONNX_OUT   = os.path.join(YOLO_DIR, "rodong.onnx")
RUN_NAME   = "rodong_yolov8n_v2"

IMGSZ = 320          # match Pi inference (yolo_node INPUT_SIZE) — do not change

# ── Training hyperparameters (CPU only, AVOID-focused, strong augmentation) ──
TRAIN_CFG = dict(
    model     = BASE_MODEL,
    imgsz     = IMGSZ,
    epochs    = 150,        # was still improving at 50ep previously -> increased
    batch     = 8,
    workers   = 4,
    patience  = 40,         # longer early stopping
    device    = "cpu",
    optimizer = "auto",
    # ── Augmentation (stronger than before to handle small data) ──
    mosaic      = 1.0,      # 0.5 -> 1.0
    close_mosaic= 15,       # disable mosaic for the last 15ep (fine-tuning)
    mixup       = 0.10,     # 0.0 -> 0.1
    copy_paste  = 0.10,
    degrees     = 5.0,
    translate   = 0.10,
    scale       = 0.5,
    shear       = 2.0,
    fliplr      = 0.5,
    flipud      = 0.0,      # up/down matters for stairs/ramps -> disabled
    hsv_h       = 0.015,
    hsv_s       = 0.7,      # 0.5 -> 0.7 (more color variation)
    hsv_v       = 0.4,      # brightness variation (for indoor lighting)
    erasing     = 0.4,      # random erasing
)


def _resolved_data_yaml():
    """Avoid the '../' relative-path issue in Roboflow's data.yaml by generating an absolute-path yaml."""
    import yaml
    src = os.path.join(DATA_DIR, "data.yaml")
    if not os.path.exists(src):
        sys.exit("ERROR: %s not found (check the dataset)" % src)
    with open(src) as f:
        d = yaml.safe_load(f)
    d["path"]  = DATA_DIR
    d["train"] = "train/images"
    d["val"]   = "valid/images"
    d["test"]  = "test/images"
    out = os.path.join(DATA_DIR, "data_local.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
    return out


def _yolo():
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ERROR: run 'pip install ultralytics' first (rodong_train_venv recommended).")
    return YOLO


def step_train(epochs=None, smoke=False):
    YOLO = _yolo()
    data = _resolved_data_yaml()
    cfg  = dict(TRAIN_CFG)
    if epochs is not None:
        cfg["epochs"] = epochs

    project = RUNS_DIR
    name    = RUN_NAME
    if smoke:
        cfg["epochs"] = 1
        project = "/tmp/rodong_retrain_smoke"
        name    = "check"
        if os.path.isdir(project):
            shutil.rmtree(project, ignore_errors=True)

    print("[train] data=%s\n[train] epochs=%d imgsz=%d device=%s"
          % (data, cfg["epochs"], cfg["imgsz"], cfg["device"]))
    model = YOLO(cfg.pop("model"))
    model.train(data=data, project=project, name=name, exist_ok=True, **cfg)
    print("[train] done -> %s/%s/" % (project, name))
    if smoke:
        shutil.rmtree(project, ignore_errors=True)
        print("[smoke] pipeline OK. Deleted temp results.")


def step_resume():
    """Resume training from last.pt after a reboot/interruption (ultralytics resume).
    It uses the saved args as-is, so any other hyperparameters are ignored."""
    YOLO = _yolo()
    last = os.path.join(RUNS_DIR, RUN_NAME, "weights", "last.pt")
    if not os.path.exists(last):
        sys.exit("ERROR: no resume target: %s (run --train first)" % last)
    print("[resume] resuming from %s" % last)
    model = YOLO(last)
    model.train(resume=True)
    print("[resume] training complete")


def step_eval():
    YOLO = _yolo()
    data = _resolved_data_yaml()
    best = os.path.join(RUNS_DIR, RUN_NAME, "weights", "best.pt")
    if not os.path.exists(best):
        sys.exit("ERROR: %s not found. Run --train first." % best)
    print("[eval] evaluating test split: %s" % best)
    model = YOLO(best)
    metrics = model.val(data=data, split="test", imgsz=IMGSZ, device="cpu")
    print("[eval] mAP50=%.3f  mAP50-95=%.3f" % (metrics.box.map50, metrics.box.map))


def step_export():
    YOLO = _yolo()
    best = os.path.join(RUNS_DIR, RUN_NAME, "weights", "best.pt")
    if not os.path.exists(best):
        sys.exit("ERROR: %s not found. Run --train first." % best)
    print("[export] %s -> ONNX (opset12, imgsz=%d)..." % (best, IMGSZ))
    model = YOLO(best)
    out = model.export(format="onnx", imgsz=IMGSZ, opset=12,
                       simplify=True, dynamic=False)
    src = out if (isinstance(out, str) and os.path.exists(out)) \
        else best.replace(".pt", ".onnx")
    if os.path.exists(src):
        shutil.copy(src, ONNX_OUT)
    print("[export] done: %s" % ONNX_OUT)
    print("[export] deploy to Pi:")
    print("  scp '%s' pi@192.168.10.2:~/xycar_ws/src/rodong/models/rodong.onnx" % ONNX_OUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train",  action="store_true")
    ap.add_argument("--eval",   action="store_true")
    ap.add_argument("--export", action="store_true")
    ap.add_argument("--all",    action="store_true", help="train→export→eval")
    ap.add_argument("--resume", action="store_true", help="resume last.pt->export->eval (after reboot)")
    ap.add_argument("--smoke",  action="store_true", help="1-epoch pipeline check")
    ap.add_argument("--epochs", type=int, default=None)
    args = ap.parse_args()

    if args.smoke:
        step_train(smoke=True)
    elif args.resume:
        step_resume(); step_export(); step_eval()
    elif args.all:
        step_train(args.epochs); step_export(); step_eval()
    elif args.train:
        step_train(args.epochs)
    elif args.export:
        step_export()
    elif args.eval:
        step_eval()
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
