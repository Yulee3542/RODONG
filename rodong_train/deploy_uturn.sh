#!/bin/bash
# deploy_uturn.sh  —  유턴 패치 Pi 배포 스크립트
# 미니PC에서 실행: bash deploy_uturn.sh

PI="pi@192.168.10.2"
SCRIPTS="~/xycar_ws/src/rodong/scripts"
PATCH="uturn_patch.py"

set -e

echo "=== [1] LAN 연결 확인 ==="
ping -c 1 -W 2 192.168.10.2 > /dev/null || {
    echo "[ERR] Pi에 연결되지 않음. LAN 설정 먼저:"
    echo "  sudo ip addr flush dev eno1"
    echo "  sudo ip addr add 192.168.10.1/24 dev eno1"
    echo "  sudo ip link set eno1 up"
    exit 1
}
echo "Pi 연결 OK"

echo "=== [2] 패치 파일 업로드 ==="
scp uturn_patch.py ${PI}:/tmp/${PATCH}
echo "업로드 완료"

echo "=== [3] 패치 적용 ==="
ssh ${PI} "python3 /tmp/${PATCH}"

echo "=== [4] catkin_make ==="
ssh ${PI} "cd ~/xycar_ws && catkin_make -DCATKIN_BLACKLIST_PACKAGES='test_' 2>&1 | tail -5"

echo ""
echo "=== 완료 ==="
echo "Pi에서 확인:"
echo "  roslaunch rodong rodong.launch"
echo "  u 키로 UTURN 테스트"
