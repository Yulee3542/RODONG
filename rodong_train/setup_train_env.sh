#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════
# RODONG12 학습 환경 셋업 (미니PC i5-6500T, CPU 전용)
#   - venv 생성 → CPU PyTorch → ultralytics → roboflow + onnx
# 사용법: bash setup_train_env.sh
# ════════════════════════════════════════════════════════════
set -e

VENV=~/rodong_train_venv

echo "═══ 1. venv 생성 ═══"
python3 -m venv "$VENV"
source "$VENV/bin/activate"
pip install --upgrade pip wheel

echo
echo "═══ 2. CPU PyTorch 설치 (GPU 없음) ═══"
# CPU 전용 wheel (용량 작고 i5에서 동작)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

echo
echo "═══ 3. ultralytics + roboflow + onnx ═══"
pip install ultralytics roboflow onnx onnxsim

echo
echo "═══ 4. 설치 확인 ═══"
python3 -c "import torch; print('torch:', torch.__version__, '| CUDA:', torch.cuda.is_available())"
python3 -c "import ultralytics; print('ultralytics:', ultralytics.__version__)"
python3 -c "import roboflow; print('roboflow OK')"
python3 -c "import onnx; print('onnx:', onnx.__version__)"

echo
echo "═══ 완료 ═══"
echo "다음:"
echo "  source $VENV/bin/activate"
echo "  export ROBOFLOW_API_KEY='발급받은_키'"
echo "  python3 rodong_train.py --download"
echo "  python3 rodong_train.py --train"
echo "  python3 rodong_train.py --export"
