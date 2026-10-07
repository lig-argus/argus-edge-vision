#!/usr/bin/env bash
set -u

echo "===== 서비스 상태 ====="
systemctl --no-pager --full status argus-busd.service argus-vision.service || true

echo
echo "===== 최근 비전 로그 60줄 ====="
journalctl --no-pager -u argus-vision.service -n 60 || true
