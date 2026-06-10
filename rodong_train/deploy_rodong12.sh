#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════
# DEPRECATED — RODONG12 era deploy.
# The Pi code was refactored into a modular layout under
# rodong_pi_code/patch(20260602)/, and the old per-file upload list
# (vfh_planner, rodong_main, yolo_node, rodong.launch) is no longer valid.
#
# This script now redirects to the single canonical deploy:
#   deploy_rodong13.sh  (uploads all nodes + core modules + launch + CMakeLists)
# ════════════════════════════════════════════════════════════
set -e
echo "[deploy_rodong12] DEPRECATED -> running deploy_rodong13.sh instead."
exec bash "$(cd "$(dirname "$0")" && pwd)/deploy_rodong13.sh" "$@"
