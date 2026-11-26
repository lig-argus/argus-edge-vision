#!/usr/bin/env bash
# 2026-11-15 사용자 지시: 라우터 자동 재시작. FC가 재부팅되면(배터리 연결 등으로 USB 재열거) mavlink-router가
# "Critical fd got error, exiting"으로 끝나고, 지금까지는 GUI를 다시 연결해야만 다시 떴다(그동안 게이트웨이
# "FC 미연결", 대기 명령 만료, 이륙 확인 시간 초과). 이제 라우터가 끝나면 FC 앱 USB 장치가 다시 나타날
# 때까지 기다렸다가 같은 설정으로 다시 띄운다. PX4 부트로더 단계(약 4.6초, 다른 USB ID)에는 열지 않는다.
# 이 스크립트를 SIGTERM/SIGINT/SIGHUP으로 끝내면 라우터도 함께 끝내고 다시 띄우지 않는다.
# 시작 시 UART를 다른 프로세스(직접 연결 게이트웨이)가 쓰고 있으면 지금처럼 바로 끝낸다.
set -Eeuo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: run_router.sh [mavlink-router arguments]. Explicit invocation opens the configured UART.'
  echo 'The router is restarted when the FC USB device comes back after a reboot.'
  exit 0
fi
source "$root/runtime_settings.sh"
router=$(command -v mavlink-routerd) || { echo 'mavlink-routerd is not installed.' >&2; exit 2; }
command -v flock >/dev/null
command -v fuser >/dev/null
mkdir -p "$root/run/router/empty.d"
exec 9>"$root/run/router/router.lock"
flock -n 9 || { echo 'This router is already running.' >&2; exit 2; }
if fuser -s "$ARGUS_UART_DEVICE" 2>/dev/null; then
  echo 'UART is in use. Disconnect the direct gateway before starting the router.' >&2
  exit 2
fi

# PX4 부트로더의 USB vendor:product (공백 구분). 이 ID일 때는 라우터를 열지 않는다.
# 기본값은 이 기체 FC(KoaFC) 부트로더 3162:004b (앱은 1b8c:0036), 2026-11-15 Pi 커널 기록 확인.
bootloader_ids=${ARGUS_FC_BOOTLOADER_USB_IDS:-3162:004b}
child=0

stop() {
  trap - TERM INT HUP
  if ((child > 0)); then
    kill -TERM "$child" 2>/dev/null || true
    wait "$child" 2>/dev/null || true
  fi
  exit 0
}
trap stop TERM INT HUP

note() {
  echo "$(date '+%F %T') run_router: $*" >&2
}

# UART tty의 USB vendor:product. 알 수 없으면 빈 값.
usb_id() {
  local name dir
  name=$(basename -- "$(readlink -f -- "$ARGUS_UART_DEVICE")")
  dir=$(readlink -f -- "${ARGUS_SYSFS_ROOT:-/sys}/class/tty/$name/device" 2>/dev/null) || return 0
  while [[ -n $dir && $dir != / && ! -f $dir/idVendor ]]; do
    dir=$(dirname -- "$dir")
  done
  if [[ -f $dir/idVendor && -f $dir/idProduct ]]; then
    echo "$(<"$dir/idVendor"):$(<"$dir/idProduct")"
  fi
}

# FC 앱 USB 장치가 있고(부트로더 아님) 다른 프로세스가 쓰지 않으면 성공.
fc_ready() {
  local id
  [[ -c $ARGUS_UART_DEVICE ]] || return 1
  id=$(usb_id)
  if [[ -n $id && " $bootloader_ids " == *" $id "* ]]; then
    return 1
  fi
  ! fuser -s "$ARGUS_UART_DEVICE" 2>/dev/null
}

wait_for_fc() {
  local waited=0
  until fc_ready; do
    if ((waited == 0)); then
      note "FC USB(앱) 대기: $ARGUS_UART_DEVICE"
    fi
    waited=1
    sleep 0.5
  done
  if ((waited)); then
    sleep 1  # 앱 USB 열거 직후 안정화
  fi
}

starts=0
while true; do
  wait_for_fc
  if ! fc_ready; then
    continue
  fi
  starts=$((starts + 1))
  if ((starts > 1)); then
    note "라우터 다시 시작 (${starts}번째)"
  fi
  # TCP stays disabled, loopback endpoints retained, host-wide config snippets excluded.
  "$router" -c "$ARGUS_ROUTER_CONFIG" -d "$root/run/router/empty.d" "$@" &
  child=$!
  code=0
  wait "$child" || code=$?
  child=0
  note "라우터 종료 (코드 $code) · FC USB가 다시 나타나면 재시작"
  sleep 1  # 바로 반복하지 않는다
done
