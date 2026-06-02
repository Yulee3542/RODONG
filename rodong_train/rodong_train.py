#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rodong_train.py  (RODONG12 - YOLO 학습 파이프라인, from scratch)
================================================================
데이터셋: Roboflow "Stairs & ramps" (Vasile Grosu)
  - ramp  → CLIMB (class 0)
  - stairs→ AVOID (class 1)
환경: 미니PC i5-6500T (CPU 전용, GPU 없음)
목표: YOLOv8n 2-class 학습 → ONNX export → Pi 배포

CPU 학습 최적화 원칙:
  - 입력 320 (640 대비 ~4배 빠름)
  - epochs 적게 (50), batch 작게 (8)
  - workers 줄임 (4코어), augmentation 가볍게
  - nano 모델 (yolov8n)

사용 순서:
  1) pip install ultralytics roboflow
  2) Roboflow에서 API 키 발급 (무료): https://app.roboflow.com → Settings → API
  3) ROBOFLOW_API_KEY 환경변수 설정 or 아래 직접 입력
  4) python3 rodong_train.py --download   # 데이터셋 다운로드 + 클래스 재매핑
  5) python3 rodong_train.py --train      # CPU 학습
  6) python3 rodong_train.py --export     # ONNX export (Pi용)
  7) python3 rodong_train.py --all        # 위 전부 순차 실행
"""

import os
import sys
import argparse
import shutil
import glob

# ── 설정 ─────────────────────────────────────────────────────────
ROBOFLOW_WORKSPACE = "vasile-grosu-uslqx"
ROBOFLOW_PROJECT   = "stairs-ramps"
ROBOFLOW_VERSION   = 1            # 첫 실행 시 사이트에서 최신 버전 확인 후 조정

# 결과물 경로
WORK_DIR   = os.path.expanduser("~/2. OWOD for Rodong/rodong_yolo")
DATA_DIR   = os.path.join(WORK_DIR, "dataset")
RUNS_DIR   = os.path.join(WORK_DIR, "runs")
ONNX_OUT   = os.path.join(WORK_DIR, "rodong.onnx")

# 클래스 매핑: Roboflow 원본 클래스명 → RODONG 클래스 ID
# (다운로드 후 data.yaml 의 names 를 보고 정확한 원본명에 맞춰 조정)
CLASS_MAP = {
    "ramp":   0,   # CLIMB
    "ramps":  0,
    "stair":  1,   # AVOID
    "stairs": 1,
}
RODONG_NAMES = ["CLIMB", "AVOID"]   # 최종 2-class

# 학습 하이퍼파라미터 (CPU 전용)
TRAIN_CFG = dict(
    model    = "yolov8n.pt",   # nano (가장 가벼움)
    imgsz    = 320,            # CPU 가속을 위해 작게
    epochs   = 50,
    batch    = 8,
    workers  = 4,              # i5 4코어
    patience = 15,             # early stopping
    device   = "cpu",
    optimizer= "auto",
    # 가벼운 augmentation (CPU 부담 ↓)
    mosaic   = 0.5,
    mixup    = 0.0,
    hsv_h    = 0.015,
    hsv_s    = 0.5,
    hsv_v    = 0.4,
)


# ────────────────────────────────────────────────────────────────
def step_download():
    """Roboflow 데이터셋 다운로드 + RODONG 2-class 재매핑."""
    try:
        from roboflow import Roboflow
    except ImportError:
        sys.exit("ERROR: pip install roboflow 먼저 실행하세요.")

    api_key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
    if not api_key:
        sys.exit("ERROR: ROBOFLOW_API_KEY 환경변수를 설정하세요.\n"
                 "  export ROBOFLOW_API_KEY='your_key_here'")

    os.makedirs(WORK_DIR, exist_ok=True)
    print(f"[download] Roboflow에서 {ROBOFLOW_PROJECT} v{ROBOFLOW_VERSION} 받는 중...")

    rf = Roboflow(api_key=api_key)
    proj = rf.workspace(ROBOFLOW_WORKSPACE).project(ROBOFLOW_PROJECT)
    dataset = proj.version(ROBOFLOW_VERSION).download("yolov8", location=DATA_DIR)

    print(f"[download] 완료: {DATA_DIR}")
    print("[download] data.yaml 의 names 를 확인하고 CLASS_MAP 을 맞추세요:")
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
    """원본 클래스 ID → RODONG ID 로 라벨 파일 재작성.
    원본 data.yaml 의 names 순서를 읽어 CLASS_MAP 기준으로 변환."""
    import yaml
    yml = os.path.join(DATA_DIR, "data.yaml")
    with open(yml) as f:
        cfg = yaml.safe_load(f)
    orig_names = cfg.get("names", [])
    print(f"[remap] 원본 클래스: {orig_names}")

    # 원본 ID → RODONG ID 변환표
    id_map = {}
    for orig_id, name in enumerate(orig_names):
        key = str(name).strip().lower()
        if key in CLASS_MAP:
            id_map[orig_id] = CLASS_MAP[key]
        else:
            id_map[orig_id] = None   # 버릴 클래스
    print(f"[remap] ID 변환표 (원본→RODONG, None=제거): {id_map}")

    # 모든 split 의 라벨 .txt 재작성
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
        print(f"[remap] {split}: {n_files}파일, {n_boxes}박스 유지, {n_dropped}박스 제거")

    # data.yaml 의 names 를 RODONG 2-class 로 덮어쓰기
    cfg["names"] = RODONG_NAMES
    cfg["nc"]    = len(RODONG_NAMES)
    with open(yml, "w") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True)
    print(f"[remap] data.yaml 갱신: nc={len(RODONG_NAMES)}, names={RODONG_NAMES}")


def step_train():
    """CPU 학습."""
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ERROR: pip install ultralytics 먼저 실행하세요.")

    yml = os.path.join(DATA_DIR, "data.yaml")
    if not os.path.exists(yml):
        sys.exit(f"ERROR: {yml} 없음. --download 먼저 실행하세요.")

    print("[train] CPU 학습 시작 (시간 오래 걸림)...")
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
    print(f"[train] 완료. 결과: {RUNS_DIR}/rodong_yolov8n/")


def step_export():
    """best.pt → ONNX export (Pi OpenCV DNN 호환)."""
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("ERROR: pip install ultralytics 먼저 실행하세요.")

    best = os.path.join(RUNS_DIR, "rodong_yolov8n", "weights", "best.pt")
    if not os.path.exists(best):
        sys.exit(f"ERROR: {best} 없음. --train 먼저 실행하세요.")

    print(f"[export] {best} → ONNX...")
    model = YOLO(best)
    # opset 12 = OpenCV DNN 호환성 좋음. simplify 로 그래프 단순화.
    out = model.export(
        format   = "onnx",
        imgsz    = TRAIN_CFG["imgsz"],
        opset    = 12,
        simplify = True,
        dynamic  = False,   # Pi OpenCV DNN 은 고정 입력이 안전
    )
    # export 결과를 ONNX_OUT 으로 복사
    if isinstance(out, str) and os.path.exists(out):
        shutil.copy(out, ONNX_OUT)
    else:
        src = best.replace(".pt", ".onnx")
        if os.path.exists(src):
            shutil.copy(src, ONNX_OUT)
    print(f"[export] 완료: {ONNX_OUT}")
    print(f"[export] Pi로 복사:")
    print(f"  scp '{ONNX_OUT}' pi@192.168.10.2:~/xycar_ws/src/rodong/models/rodong.onnx")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--download", action="store_true", help="데이터셋 다운로드+재매핑")
    ap.add_argument("--train",    action="store_true", help="CPU 학습")
    ap.add_argument("--export",   action="store_true", help="ONNX export")
    ap.add_argument("--all",      action="store_true", help="전부 순차 실행")
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
