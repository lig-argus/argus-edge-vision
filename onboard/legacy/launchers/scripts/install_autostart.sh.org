#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON_BIN="$PROJECT_DIR/.venv-rpi/bin/python"
TEMPLATE_DIR="$PROJECT_DIR/deploy/systemd"
RUNTIME_ENV="$PROJECT_DIR/config/runtime.env"
RUN_USER="${SUDO_USER:-$USER}"

if [[ "$RUN_USER" == "root" ]]; then
  echo "[오류] 일반 사용자로 로그인한 뒤 이 스크립트를 실행하십시오." >&2
  exit 1
fi
if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "[오류] bash scripts/setup_rpi.sh를 먼저 실행하십시오." >&2
  exit 1
fi
if [[ ! -f "$RUNTIME_ENV" ]]; then
  echo "[오류] $RUNTIME_ENV 파일이 없습니다." >&2
  echo "cp config/runtime.env.example config/runtime.env 후 모델 경로를 수정하십시오." >&2
  exit 1
fi
if [[ "$PROJECT_DIR" == *$'\n'* ]]; then
  echo "[오류] 프로젝트 경로에 줄바꿈 문자를 사용할 수 없습니다." >&2
  exit 1
fi

echo "[1/4] 카메라, Hailo-8, HEF와 라벨을 사전 검사합니다."
bash "$SCRIPT_DIR/service_entrypoint.sh" check

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf -- "$tmp_dir"
}
trap cleanup EXIT

render_unit() {
  local source="$1"
  local output="$2"
  "$PYTHON_BIN" - "$source" "$output" "$PROJECT_DIR" "$RUN_USER" <<'PY'
from pathlib import Path
import sys

source, output, project_dir, run_user = sys.argv[1:]
text = Path(source).read_text(encoding="utf-8")
text = text.replace("@PROJECT_DIR@", project_dir).replace("@RUN_USER@", run_user)
Path(output).write_text(text, encoding="utf-8")
PY
}

echo "[2/4] systemd 서비스 파일을 생성합니다."
render_unit "$TEMPLATE_DIR/argus-busd.service.in" "$tmp_dir/argus-busd.service"
render_unit "$TEMPLATE_DIR/argus-vision.service.in" "$tmp_dir/argus-vision.service"

echo "[3/4] 서비스를 설치하고 부팅 자동 실행을 활성화합니다."
"${SUDO[@]}" install -m 0644 "$tmp_dir/argus-busd.service" /etc/systemd/system/argus-busd.service
"${SUDO[@]}" install -m 0644 "$tmp_dir/argus-vision.service" /etc/systemd/system/argus-vision.service
"${SUDO[@]}" systemctl daemon-reload
"${SUDO[@]}" systemctl enable --now argus-busd.service argus-vision.service

echo "[4/4] 서비스 상태를 확인합니다."
"${SUDO[@]}" systemctl --no-pager --full status argus-busd.service argus-vision.service || true

cat <<EOF

부팅 자동 실행 설치가 완료되었습니다.

상태 확인:
  bash scripts/autostart_status.sh

실시간 로그:
  journalctl -u argus-vision.service -f

설정 변경:
  nano "$RUNTIME_ENV"
  sudo systemctl restart argus-vision.service
EOF
