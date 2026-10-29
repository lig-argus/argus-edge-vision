# mavlink-router 설정

RPi 에서 FC 의 UART 를 직접 읽고 쓰는 프로세스는 mavlink-router 하나이고, onboard(UDP 14540), 통제기(UDP 14550), QGC(UDP 14551)로 중계한다 (`docs/spec/icd.md` 1.2).

| 파일 | 용도 |
|---|---|
| `main.conf` | 비행 설정. `/etc/mavlink-router/main.conf` 에 설치한다 |
| `config.d/qgc-maintenance.conf` | QGC 로 PX4 파라미터, 보정을 할 때만 `/etc/mavlink-router/config.d/` 에 추가한다. 비행 전에 삭제한다 |

## 설치 (RPi, Ubuntu 24.04)

mavlink-router 는 apt 패키지가 없다. mavlink-router 저장소 README 가 안내하는 방법은 meson 과 ninja 로 소스를 빌드하는 것이고(ArduPilot 문서도 이 README 를 가리킨다), 그대로 따른다. GitHub 릴리스에 미리 빌드된 바이너리(`mavlink-routerd-glibc-aarch64`)도 있지만 README 에 안내되지 않은 경로라 사용하지 않는다.

```bash
sudo apt install git meson ninja-build pkg-config gcc g++ systemd
git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
meson setup build . --buildtype=release -Dsystemdsystemunitdir=/usr/lib/systemd/system
ninja -C build
sudo ninja -C build install
```

`ninja install` 이 `mavlink-routerd` 와 systemd unit `mavlink-router.service` 를 설치한다. unit 은 `/etc/mavlink-router/main.conf` 와 `/etc/mavlink-router/config.d/` 를 읽는다.

UART `/dev/ttyAMA0` 를 mavlink-router 가 사용하려면 리눅스의 시리얼 로그인 셸을 비활성화하고 UART 하드웨어를 활성화해야 한다. PX4 와 ArduPilot 문서의 절차:

```bash
sudo apt install raspi-config
sudo raspi-config
```

Interface Options → Serial Port 에서 "login shell over serial" 은 No, "serial port hardware" 는 Yes 를 선택한다. 이어서 `/boot/firmware/config.txt` 끝에 다음 두 줄을 추가하고 재부팅한 뒤 `ls /dev/ttyAMA0` 로 장치를 확인한다.

```
enable_uart=1
dtoverlay=disable-bt
```

`/dev/serial0` 은 같은 장치를 가리키는 별칭이다. 설정 파일과 `docs/spec/icd.md` 1.2 는 `/dev/ttyAMA0` 을 사용한다.

출처: PX4 user guide "Raspberry Pi Companion with Pixhawk" (Ubuntu 에서 `raspi-config` 설치와 `config.txt` 두 줄), ArduPilot wiki "Communicating with Raspberry Pi via MAVLink" (`raspi-config` 의 두 선택, mavlink-router `Baud = 921600`).

설정 파일 설치 (저장소 루트에서). `mavlink-routerd` 를 직접 실행하고 있으면 그 프로세스를 먼저 종료한다. 같은 UART 와 포트를 두 개가 열 수 없다.

```bash
sudo mkdir -p /etc/mavlink-router/config.d
sudo install -m 644 onboard/config/mavlink-router/main.conf /etc/mavlink-router/main.conf
sudo systemctl enable --now mavlink-router
```

## 확인

```bash
systemctl status mavlink-router
sudo journalctl -u mavlink-router -n 30
ss -ulnp | grep mavlink-routerd
```

- `ss` 에 14550, 14551 이 표시되어야 한다 (14540 은 onboard 가 여는 포트라 라우터 쪽에는 표시되지 않는다).
- 통제기를 RPi Wi-Fi 에 붙이고 14550 으로 HEARTBEAT 를 보내면 FC telemetry 가 통제기에 도착한다. 통제기가 먼저 보내야 라우터가 통제기 주소를 인식한다.
- onboard 를 실행하면 14540 으로 FC 메시지가 수신된다.
- QGC 입력 차단: 비행 설정(`main.conf` 만)에서 QGC 로 파라미터 하나를 바꾸면 FC 에 반영되지 않고, 정비 설정(`config.d/qgc-maintenance.conf` 추가)에서는 반영되어야 한다. 기체에서 확인한다.

## QGC 로 파라미터, 보정하기 (정비)

비행 설정에서는 QGC(sysid 255)가 보낸 패킷을 라우터가 모두 폐기한다(`main.conf` 의 `BlockSrcSysIn = 255`). QGC 는 telemetry 만 수신할 수 있다. 파라미터와 보정은 아래 절차로 한다.

정비 시작:

```bash
sudo install -m 644 onboard/config/mavlink-router/config.d/qgc-maintenance.conf /etc/mavlink-router/config.d/
sudo systemctl restart mavlink-router
```

비행 전:

```bash
sudo rm /etc/mavlink-router/config.d/qgc-maintenance.conf
sudo systemctl restart mavlink-router
ls /etc/mavlink-router/config.d/
```

`ls` 에 아무것도 나오지 않아야 비행 설정이다. `main.conf` 는 두 상태에서 같고 편집하지 않는다.

이유: 비행 명령(시동, 이륙, 모드 변경)과 보정 명령은 모두 같은 `COMMAND_LONG` 메시지로 가고, mavlink-router 는 메시지 id 와 보낸 쪽 sysid 로만 필터링할 수 있어 "보정은 통과, 시동은 차단" 을 구현할 수 없다. 그래서 비행 중에는 QGC 가 보낸 패킷을 모두 차단하고, 정비 때만 허용한다 (ArduPilot 이 FC 에서 하는 GCS sysid 검사와 같은 방식. PX4 에는 그 기능이 없어 라우터에서 수행한다).

## 네트워크 조건

14550 과 14551 은 RPi 의 모든 인터페이스(`0.0.0.0`)에서 받으므로, 같은 Wi-Fi 에 있는 어느 장치든 MAVLink 를 보낼 수 있다. 기체와 통제기만 있는 사설 Wi-Fi 에서만 사용한다. 근본 보호는 MAVLink 서명이며 아직 사용하지 않는다 (`docs/spec/icd.md` 7장 D1).

## 설정을 바꿀 때

- 포트, 주소, 장치를 바꾸면 `docs/spec/icd.md` 1.2 링크 표도 함께 수정한다.
- 주석은 줄 전체로만 작성한다. mavlink-router 는 줄 끝 주석과 빈 값(`Key =`)을 오류로 거부한다.
- 키 전체 목록은 mavlink-router 저장소의 `examples/config.sample`.
