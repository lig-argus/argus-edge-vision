#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" systemctl disable --now argus-vision.service argus-busd.service 2>/dev/null || true
"${SUDO[@]}" rm -f /etc/systemd/system/argus-vision.service /etc/systemd/system/argus-busd.service
"${SUDO[@]}" systemctl daemon-reload
"${SUDO[@]}" systemctl reset-failed

echo "ARGUS 부팅 자동 실행을 제거했습니다. 프로젝트와 모델 파일은 삭제하지 않았습니다."
