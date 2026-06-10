#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════
# RODONG12 training environment setup (mini PC i5-6500T, CPU only)
#   - create venv -> CPU PyTorch -> ultralytics -> roboflow + onnx
# Usage: bash setup_train_env.sh
# ════════════════════════════════════════════════════════════
set -e

VENV=~/rodong_train_venv

echo "═══ 1. Create venv ═══"
python3 -m venv "$VENV"
source "$VENV/bin/activate"
pip install --upgrade pip wheel

echo
echo "═══ 2. Install CPU PyTorch (no GPU) ═══"
# CPU-only wheel (small and runs on the i5)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

echo
echo "═══ 3. ultralytics + roboflow + onnx ═══"
pip install ultralytics roboflow onnx onnxsim

echo
echo "═══ 4. Verify installation ═══"
python3 -c "import torch; print('torch:', torch.__version__, '| CUDA:', torch.cuda.is_available())"
python3 -c "import ultralytics; print('ultralytics:', ultralytics.__version__)"
python3 -c "import roboflow; print('roboflow OK')"
python3 -c "import onnx; print('onnx:', onnx.__version__)"

echo
echo "═══ Done ═══"
echo "Next:"
echo "  source $VENV/bin/activate"
echo "  export ROBOFLOW_API_KEY='your_issued_key'"
echo "  python3 rodong_train.py --download"
echo "  python3 rodong_train.py --train"
echo "  python3 rodong_train.py --export"
