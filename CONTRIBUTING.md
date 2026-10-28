# Contributing Guide

argus-edge-vision 프로젝트의 브랜치, 커밋, PR 규칙이다. 작업 전에 한 번 읽는다.

## 0.1 처음 한 번만 — 로컬 커밋 검사 켜기

커밋 메시지는 [commitlint](https://commitlint.js.org) 로 검사하며, [husky](https://typicode.github.io/husky/) 가 git hook 을 설치한다.

**준비물: Node.js 22.12 이상** (`node --version` 으로 확인)

| OS | 설치 |
|---|---|
| macOS | `brew install node@22` |
| Windows | `winget install OpenJS.NodeJS.LTS` |
| Ubuntu | [NodeSource](https://github.com/nodesource/distributions) 또는 `nvm install 22` |

clone 한 뒤 저장소 루트에서 한 번만 실행한다.

```bash
npm install
```

- `commitlint`, `husky` 가 `node_modules/` 에 설치되고, `commit-msg` hook 이 자동으로 연결된다.
- 이후 규칙에 맞지 않는 커밋 메시지는 `git commit` 시점에 거부된다. (터미널, VS Code, CLion 등 어떤 도구로 커밋해도 동일)
- `package.json` 은 **커밋 메시지 검사 도구 전용**이다. 프로젝트 코드(C/C++, Python)와는 무관하다.
- `git commit --no-verify` 로 검사를 건너뛰지 않는다. 건너뛰어도 PR 에서 CI 에 다시 걸린다.

PR 에서는 GitHub Actions(`commitlint`)가 `package-lock.json` 에 고정된 **같은 버전**의 commitlint 로 한 번 더 검사하며, 통과해야 머지할 수 있다.

---

## 0.2 처음 한 번만 — clang-format 과 clang-tidy 설치

C++ 형식은 clang-format 23, 코드 검사는 clang-tidy 로 한다. 규칙은 `.clang-format`(Google 스타일 + 6항목)과 `.clang-tidy`, 통제기는 `gcs/.clang-tidy`(이름 규칙만 Qt camelCase)이다.

| OS | 설치 |
|---|---|
| macOS | `brew install clang-format llvm`, `.zshrc` 에 `export PATH="/opt/homebrew/opt/llvm/bin:$PATH"` |
| Linux (Ubuntu 24.04) | `pip install clang-format==23.1.3 clang-tidy==22.1.8` (CI 와 같은 버전) |
| Windows | [Git for Windows](https://git-scm.com) + `pip install clang-format==23.1.3 clang-tidy==22.1.8`. 명령은 Git Bash 에서 실행한다 |

- clang-format 은 23 이어야 한다 (버전마다 결과가 다르다). `apt install clang-tidy` 는 18 이라 `gcs/.clang-tidy` 를 읽지 못하므로 사용하지 않는다.
- pip 으로 설치하면 `run-clang-tidy` 대신 `run-clang-tidy.py` 이다. `clang-format` 이 PATH 에 있어야 hook 이 찾는다.
- 커밋할 때 `.husky/pre-commit` 이 스테이징한 C++ 파일의 형식을 검사한다. 거부되면 `clang-format -i <파일>` 로 고친 뒤 다시 `git add` 한다. PR 에서는 CI(`.github/workflows/ci.yml`)가 형식을 다시 검사한다. 빌드, 테스트, clang-tidy 는 CI 의 `gcs` 작업과 `onboard` 작업이 한다.

**C++ 이름 규칙**

`.clang-tidy` 의 `readability-identifier-naming` 이 아래 규칙을 검사한다. 통제기(`gcs/`)는 Qt 에 맞춰 함수와 변수만 camelCase 이다 (`gcs/.clang-tidy`).

| 대상 | 규칙 | 예 |
|---|---|---|
| 클래스, 구조체, enum, 타입 별칭 | `CamelCase` | `VehicleState`, `CommandId` |
| 인터페이스 (순수 가상 함수만 있는 클래스) | `I` + `CamelCase` | `IVehicleTelemetry` |
| enum 값 | `CamelCase` | `FlightMode::Hold`, `FollowOutcome::InProgress` |
| 함수, 메서드 | `snake_case` | `attitude_at()`, `request_hold()` |
| 변수, 매개변수, 구조체의 public 멤버 | `snake_case` | `valid_until_monotonic_ns` |
| private 와 protected 멤버 변수 | `snake_case` + `_` | `latest_`, `handlers_` |
| 상수 (`constexpr`, 전역, static) | `k` + `CamelCase` | `kAttitudeMatchToleranceNs` |
| 매크로 | `UPPER_CASE` | `ARGUS_ENABLE_TRACE` |
| 헤더 가드 | include 경로를 대문자로, `/` 와 `.` 은 `_`, 끝에 `_` (Google 방식) | `argus/vehicle_api/types.hpp` → `ARGUS_VEHICLE_API_TYPES_HPP_` |
| namespace | `snake_case` | `argus::vehicle_api` |
| 파일 | `snake_case`, 헤더 `.hpp`, 소스 `.cpp`, 테스트 `<이름>_test.cpp` | `vehicle_telemetry.hpp` |

- 형식은 Google 스타일이지만 이름은 Google 규칙(함수 `PascalCase`)을 따르지 않는다. 함께 사용하는 라이브러리와 같은 분야 프로젝트가 함수와 변수를 snake_case 로 사용하기 때문이다: C++ 표준 라이브러리(`push_back()`), cppzmq(`send()`), MAVSDK(`Action::arm()`, `Telemetry::subscribe_position()`), ArduPilot(`get_position()`), ROS 2 `rclcpp`(`create_publisher()`).
- Python(PEP 8)도 함수와 변수가 snake_case 라서 같은 필드를 C++ 과 Python 에서 같은 이름으로 사용할 수 있다.
- 헤더는 `#ifndef`, `#define`, `#endif` include guard 를 사용하고 `#pragma once` 는 사용하지 않는다. `#pragma once` 는 C++ 표준이 아니고, 방산 코딩 표준인 JSF AV C++ Rule 27 이 include guard 외의 방법을 금지하기 때문이다.

**클래스 이름 짓기**

`.clang-tidy` 가 검사하지 않는, 이름의 뜻에 대한 규칙이다.

| 대상 | 규칙 | 예 |
|---|---|---|
| 인터페이스 | `I` + 역할 이름. 값 타입과 같은 이름은 사용하지 않는다 | `IVehicleTelemetry` (값 타입 `VehicleState` 와 구분), `IVelocitySetpointSink` (값 타입 `VelocitySetpoint` 와 구분) |
| 구현 클래스 | 방식 + 인터페이스 이름에서 `I` 를 뺀 것. 방식은 전송 방식이나 기술의 이름(`Zmq`, `Udp`, `Mavlink`, `Steady`, `Fake`)이고 대상 이름(`Fc`)은 방식이 아니다 | `ZmqRpcClient`, `UdpMavlinkConnection`, `MavlinkFlightModeControl`, `SteadyClock`, `FakeClock` |
| 테스트용 구현 | `Fake` + 인터페이스 이름에서 `I` 를 뺀 것 | `FakeMavlinkConnection`, `FakeFollowControl` |
| RPC 의 두 쪽 | 구현하는 쪽(동기, 결과를 바로 반환)은 역할 이름, 호출하는 쪽(비동기, 응답은 handler 로)은 `…Client` | `IFollowControl` / `IFollowControlClient`, `IFlightModeControl` / `IFlightModeControlClient` |
| 받는 쪽 어댑터 | 응답하면 `…Server`, 받아서 다른 곳으로 넘기면 `…Receiver`, 최신 값을 들고 나눠 주면 `…Subscriber` | `ZmqFollowControlServer`, `ZmqVelocitySetpointReceiver`, `ZmqFollowStatusSubscriber` |
| 보내는 쪽 | ZeroMQ 토픽 발행은 `…Publisher`, MAVLink 로 통제기에 보고는 `…Reporter`, 받은 값을 밖으로 내보내는 끝은 `…Sink` | `FcTelemetryPublisher`, `ArgusStatusReporter`, `MavlinkVelocitySetpointSink` |
| `Argus` 접두사 | MAVLink 메시지 이름(`ARGUS_STATUS` 등)을 딴 클래스에만 사용한다. 프로젝트 이름으로 붙이지 않는다(namespace `argus` 가 그 역할) | `ArgusStatusReporter`, `ArgusSensorStatusReporter` |
| 등록 함수 | 이름은 `subscribe…`, 반환은 `[[nodiscard]] core::Subscription` (객체가 사라지면 등록이 풀림). `on_…(handler)` 처럼 해제 수단이 없는 형태는 사용하지 않는다 | `subscribe(msgid, handler)`, `subscribe_flight_mode_events(handler)` |
| 테스트용 구현의 함수 | 메시지 수신, 응답 도착, 모드 변경처럼 바깥에서 일어나는 일을 테스트가 대신 일으키는 함수는 `simulate_` + 도착한 메시지, 응답, 이벤트의 이름. 반환값을 미리 정하는 함수는 `set_` + 그 값의 이름 (Chromium 의 Fake 관례 `Simulate…`, `Set…`) | `simulate_message_received(message)`, `simulate_frame_received(frame)`, `simulate_reply(result)`, `simulate_flight_mode_event(event)`, `set_send_result(false)` |

- 구현 이름을 "방식 + 인터페이스" 로 하는 이유: 같은 인터페이스의 구현이 여럿일 때(ZeroMQ, MAVLink, Fake) 이름만 보고 무엇을 어떻게 구현했는지 알 수 있다. MAVSDK 의 `Connection`/`UdpConnection`/`TcpConnection`, PX4 의 `MavlinkStream…` 과 같은 방식이다.

**멤버 설명 주석**

한 줄에 들어가는 짧은 사실(단위, 범위)만 선언과 같은 줄의 줄 끝 주석(trailing comment)으로 둔다. 두 줄 이상인 설명은 멤버 위 줄에 두고, 그 멤버 앞뒤를 빈 줄로 띄워 어느 멤버의 설명인지 드러낸다. 줄 끝 주석이 길면 clang-format 이 선언을 두 줄로 나누기 때문이다 (Chromium 의 방식).

**IDE 에서 형식 정리하기**

IDE 는 루트의 `.clang-format` 을 자동으로 읽어 저장할 때 또는 단축키로 정리한다. 평소에는 이것으로 충분하고, 아래 0.3 의 명령은 전체를 한 번에 확인할 때 사용한다. 단, **IDE 가 사용하는 clang-format 이 23 이 아니면 결과가 달라 hook 에서 거부**되므로 위에 설치한 clang-format 23 을 사용하도록 지정한다.

| IDE | 설정 |
|---|---|
| VS Code (C/C++ 확장) | `settings.json` 의 `C_Cpp.clang_format_path` 에 설치한 clang-format 경로. `editor.formatOnSave: true` |
| Visual Studio | 도구 > 옵션 > 텍스트 편집기 > C/C++ > 서식 > "사용자 지정 clang-format.exe 파일 사용" 에 경로 |
| CLion | Settings > Editor > Code Style > C/C++ 에서 ClangFormat 사용. 내장 버전을 사용하므로 저장 뒤 0.3 의 형식 검사로 한 번 확인 |
| Qt Creator | ClangFormat 플러그인 켜기 (Help > About Plugins). 내장 버전을 사용하므로 CLion 과 같이 확인 |

**편집기 공통 설정 (`.editorconfig`)**

저장소 루트의 `.editorconfig` 가 들여쓰기(C++ 4칸, XML 2칸), 줄 끝(LF), 파일 끝 개행, 끝 공백 제거를 정한다. 편집기가 파일을 열 때 자동으로 읽으므로 따로 설치할 것은 없고, 지원이 켜져 있는지만 확인한다.

| 편집기 | 확인 |
|---|---|
| CLion | 기본 켜짐. Settings > Editor > Code Style > "Enable EditorConfig support" |
| VS Code | 확장 "EditorConfig for VS Code" 설치 |
| Visual Studio | 기본 켜짐 (2017 이상) |
| Qt Creator | 기본 켜짐 (Preferences > Text Editor > Behavior > "Use EditorConfig") |

확인 방법: `common/mavlink/message_definitions/argus.xml` 에서 Enter 를 누르면 2칸, `.cpp` 에서는 4칸 들여쓰기가 되면 적용된 것이다.

**Python 형식 정리, 코드 검사, 타입 검사 (Ruff, mypy)**

Python 은 PEP 8 스타일을 따르고, docstring 형식만 Google 방식(`Args:`, `Returns:`, `Raises:`)을 사용한다. 형식과 규칙 검사는 Ruff, 타입 검사는 mypy(strict)로 한다. 규칙은 루트 `pyproject.toml` 이고, 도구 버전은 `tools/requirements-lint.txt` 에 고정되어 CI 와 같다.

| OS | 설치 |
|---|---|
| macOS, Linux | `python3.12 -m venv ~/.venvs/lint && ~/.venvs/lint/bin/pip install -r tools/requirements-lint.txt` 뒤 `.zshrc`(또는 `.bashrc`)에 `export PATH="$HOME/.venvs/lint/bin:$PATH"` |
| Windows | `py -3.12 -m venv %USERPROFILE%\.venvs\lint` → `%USERPROFILE%\.venvs\lint\Scripts\pip install -r tools/requirements-lint.txt`, 명령은 Git Bash 에서 실행한다 |

- `ruff` 와 `mypy` 가 PATH 에 있어야 커밋 hook 이 찾는다.
- 커밋할 때 `.husky/pre-commit` 이, 스테이징한 `.py` 가 있으면 그 파일을 `ruff format --check` 와 `ruff check` 로 검사하고, git 이 관리하는 `.py` 전체를 `mypy` 로 검사한다. 거부되면 0.3 의 정리 명령으로 수정한 뒤 다시 `git add` 한다.
- 한 줄은 88 자, 들여쓰기 4칸이다. 이름은 클래스 `CamelCase`, 함수와 변수 `snake_case`, 상수 `UPPER_CASE` (PEP 8).
- 공개 함수와 클래스에는 docstring 이 필요하다. 모듈과 패키지 docstring 과 테스트 파일은 요구하지 않는다.
- mypy 는 모든 함수에 타입 힌트를 요구한다. 타입 정보가 없는 외부 패키지(예: HailoRT Python)는 `pyproject.toml` 의 `[[tool.mypy.overrides]]` 에 패키지별로만 예외를 둔다.

| IDE | 설정 |
|---|---|
| VS Code | 확장 "Ruff"(Astral)와 "Mypy Type Checker"(Microsoft) 설치. 두 확장이 `~/.venvs/lint` 의 도구를 사용하도록 인터프리터 경로를 지정하고 `editor.formatOnSave: true` |
| PyCharm | 플러그인 "Ruff" 설치, Settings > Tools > Ruff 에서 `~/.venvs/lint/bin/ruff` 지정. 타입 검사는 PyCharm 자체 검사와 0.3 의 mypy 명령으로 확인 |

Google 형식 docstring 예:

```python
def make_header(frame_id: int, unix_ns: int) -> FrameHeader:
    """vision/frame/gray8 메시지 헤더를 만든다.

    Args:
        frame_id: 탐지 결과와 같은 프레임 번호.
        unix_ns: RPi 수신 시각.

    Returns:
        만든 헤더.

    Raises:
        ValueError: frame_id 가 음수일 때.
    """
```

## 0.3 형식 검사 직접 실행하기

저장소 루트에서 실행한다 (hook 과 CI 가 사용하는 명령).

```bash
# 형식 검사: 파일은 건드리지 않고, 고칠 자리를 "파일:줄:칸: error:" 로 출력. 고칠 곳이 있으면 exit 1 (hook 과 CI 가 사용하는 명령)
#   --dry-run 만 사용하면 warning 으로 출력하고 exit 0. -Werror 가 error 로 올려 exit 1 로 만든다
git ls-files '*.cpp' '*.hpp' '*.h' ':!common/mavlink/generated' | xargs -r clang-format --dry-run -Werror
# 형식 정리: 같은 규칙으로 계산한 결과를 파일에 덮어쓴다. 출력 없음, 항상 exit 0 (내가 고칠 때)
git ls-files '*.cpp' '*.hpp' '*.h' ':!common/mavlink/generated' | xargs -r clang-format -i

# clang-tidy: 빌드 폴더의 compile_commands.json 에 있는 소스를 검사. 경고도 실패로
run-clang-tidy -p build/onboard -quiet -warnings-as-errors='*'
run-clang-tidy -p build/gcs -quiet -warnings-as-errors='*' '^(?!.*_autogen)'   # 통제기: Qt moc 가 만든 소스 제외

# Python 형식 검사: 수정할 곳이 있으면 exit 1 (hook 과 CI 가 사용하는 명령)
ruff format --check .
# Python 형식 정리: 파일에 덮어쓴다 (직접 수정할 때)
ruff format .
# Python 규칙 검사. --fix 를 붙이면 자동으로 수정할 수 있는 것은 수정한다
ruff check .
# Python 타입 검사: git 이 관리하는 .py 전체 (hook 과 CI 가 검사하는 범위)
git ls-files -z -- '*.py' ':!common/mavlink/generated' | xargs -0 mypy
```

- clang-tidy 는 먼저 `cmake -S … -B build/…` 로 설정해 둬야 한다. macOS 는 `-DCMAKE_OSX_SYSROOT="$(xcrun --show-sdk-path)"` 를 붙이다. Windows 는 `-G Ninja` 로 설정해야 `compile_commands.json` 이 생긴다.

## 0.4 onboard 모듈 규칙

`onboard/libs/` 와 `onboard/apps/` 아래 폴더 하나가 모듈 하나이다.

| 항목 | 규칙 | 예 (`libs/bus`) |
|---|---|---|
| 폴더 | `include/, src/, test/` | `onboard/libs/bus/{include,src,test}/` |
| 헤더 경로 | `include/argus/<모듈>/` | `onboard/libs/bus/include/argus/bus/topic_publisher.hpp` |
| include | 모듈 이름부터 사용한다 | `#include "argus/bus/topic_publisher.hpp"` |
| namespace | `argus::<모듈>` | `argus::bus::ITopicPublisher` |
| CMake 타깃 | 라이브러리 `argus_<모듈>` + 별칭 `argus::<모듈>`, 실행 파일 `argus_<모듈>` | `target_link_libraries(... argus::bus)` |
| 테스트 | gtest. `test/<이름>_test.cpp` 마다 `argus_add_test` 한 번 | `argus_add_test(bus_topic_publisher_test SOURCES test/topic_publisher_test.cpp LIBS argus::bus)` |

- 헤더 경로에 `argus/<모듈>/` 을 넣는 이유: 모듈마다 같은 파일 이름(`types.hpp` 등)이 있어도 include 가 겹치지 않고, 어느 모듈의 헤더인지 include 문만 보고 알 수 있다 (ROS 2 `rclcpp` 의 `include/rclcpp/` 와 같은 방식).
- 모듈이 다른 모듈을 사용할 때는 `target_link_libraries` 로 연결한다. 다른 모듈의 `src/` 나 상대 경로(`../`)로 헤더를 부르지 않는다.
- 라이브러리는 헤더만 있으면 INTERFACE, `src/` 에 `.cpp` 가 생기면 STATIC 으로 바꾼다.
- 인터페이스만 있는 모듈(`vehicle_api`)은 통신 구현(ZeroMQ, MessagePack)에 의존하지 않고 `core`(`Subscription`)에만 의존한다. ZeroMQ 구현은 `<모듈>_zmq` 모듈(`vehicle_api_zmq`)로 따로 두어, 알고리즘 코드와 테스트용 구현이 인터페이스 모듈만 링크하게 한다.
- 테스트용 구현(`Fake…`)의 위치: 여러 모듈의 테스트가 사용하면 `<모듈>_testing` 모듈(`core_testing`, `bus_testing`, `vehicle_api_testing`)에, 한 모듈만 사용하면 그 모듈의 `test/` 에 둔다(`apps/onboard/test/fake_mavlink_connection.hpp`). 제품 실행 파일은 `*_testing` 을 링크하지 않는다.
- 모든 handler(토픽 구독, RPC 응답, MAVLink 메시지, 주기 작업)는 `core::IPoller::process_events()` 안에서, 그 함수를 부른 스레드에서만 불린다. 프로세스는 스레드 하나로 돌고, handler 는 짧게 끝내야 한다 (`docs/design/uml/class_onboard.md` 1절).
  이 규칙은 onboard 프로세스(`onboard/apps/onboard`. FC 와 통제기 통신, FC 로 가는 명령 검사를 맡는 프로세스)에 적용한다. 추론처럼 오래 걸리는 작업은 별도 프로세스(탐지)에 두고 결과만 버스로 받는다.
- 빌드와 테스트 명령, 의존성 설치는 README.md "onboard 구조와 빌드" 확인.

---

## 1. 브랜치 전략 (Git Flow)

| 브랜치 | 용도 | 소스 브랜치 | 타겟 브랜치 |
|---|---|---|---|
| `main` | 배포(릴리즈) 버전 | — | — |
| `develop` | 다음 릴리즈 통합 | `main` | — |
| `feature/<이슈번호>-<설명>` | 기능 개발 | `develop` | `develop` |
| `release/v<X.Y.Z>` | 릴리즈 준비 (버그 수정, 버전 표기만) | `develop` | `main`, `develop` |
| `hotfix/v<X.Y.Z>` | 배포 버전 긴급 수정 | `main` | `main`, `develop` |

브랜치 이름 예시

```
feature/12-onboard-frame-capture
feature/27-gcs-telemetry-view
release/v0.2.0
hotfix/v0.2.1
```

- 브랜치 이름은 **소문자 + 하이픈(-)** 만 사용한다.
- `main`, `develop` 에는 직접 push 할 수 없다. 반드시 PR로 머지한다.

---

## 2. 커밋 메시지 (Conventional Commits)

### 형식

```
<type>(<scope>): <subject>

<body>        ← 선택 사항: 무엇을, 왜 바꿨는지

<footer>      ← 선택 사항: 이슈 연결, BREAKING CHANGE
```

### type

| type | 언제 |
|---|---|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `docs` | 문서만 변경 |
| `style` | 포맷, 세미콜론 등 동작 변화 없는 코드 스타일 |
| `refactor` | 기능 변화 없는 구조 개선 |
| `perf` | 성능 개선 |
| `test` | 테스트 추가/수정 |
| `build` | 빌드 시스템, 툴체인 (CMake, Makefile 등) |
| `ci` | GitHub Actions 등 CI 설정 |
| `chore` | 그 외 변경사항 (.gitignore, 폴더 구조 등) |
| `revert` | 이전 커밋 되돌리기 |

### scope (생략 가능)

| scope | 대상 |
|---|---|
| `onboard` | `onboard/` — 기체 탑재 SW |
| `gcs` | `gcs/` — 지상 통제 SW |
| `common` | `common/` — onboard와 gcs가 함께 사용하는 라이브러리 |
| `simulation` | `simulation/` — 시뮬레이션 환경과 실험 |
| `models` | `models/` — 학습/추론 모델 (데이터셋은 저장소에 올리지 않는다) |
| `tools` | `tools/` — 생성과 검사 스크립트 |
| `docs` | `docs/` |
| `ci` | `.github/` 워크플로 |
| `deps` | 의존성 버전 변경 |

여러 영역에 걸치거나 루트 설정 파일만 바꿨다면 scope 를 생략한다.
새 scope 가 필요하면 `commitlint.config.mjs` 와 이 문서를 **함께** 수정하는 PR 을 올린다.

### subject 규칙

- 첫 줄(header)은 **72자 이내**
- `type` 과 `scope` 는 영문, **subject 는 한글**로 작성한다.
- subject 는 **`~ 추가`, `~ 수정`, `~ 삭제` 처럼 명사형**으로 끝낸다. (~~`추가했습니다`~~, ~~`추가함`~~)
- 끝에 마침표(`.`)를 붙이지 않는다.
- 본문(body)은 한 줄 **100자 이내**를 권장한다. (넘으면 경고)

### 예시

```
feat(onboard): 카메라 프레임 캡처 태스크 추가
fix(gcs): 텔레메트리 패킷이 잘릴 때 크래시 발생 문제 수정
docs: Jetson 빌드 방법 갱신
chore: 초기 프로젝트 디렉토리 구조 추가
refactor(models): 전처리 로직을 별도 모듈로 분리
feat(onboard)!: 텔레메트리 패킷 형식을 v2로 변경
```

본문과 footer 가 있는 예시

```
fix(onboard): 타임아웃 시 mutex 미해제로 태스크가 멈추는 문제 수정

타임아웃 경로에서 mutex 를 해제하지 않아 다음 주기에
detection task 가 영구 대기하는 문제 수정.

Closes #34
```

### 호환되지 않는 변경 (Breaking Change)

type 뒤에 `!` 를 붙이고 footer 에 설명을 남긴다.

```
feat(onboard)!: 텔레메트리 패킷 형식을 v2로 변경

BREAKING CHANGE: GCS 는 v2 파서로 업데이트해야 수신 가능
```

---

## 3. Pull Request

### 머지 방식

머지 방식은 **타겟 브랜치**로 정해지며, 저장소 rulesets 가 강제하므로 다른 버튼은 보이지 않는다.

| 타겟 브랜치 | 머지 방식 | 소스 브랜치 |
|---|---|---|
| `develop` | **Squash and merge** | `feature/*`, 그리고 `release/*` 와 `hotfix/*` 의 develop 반영 |
| `main` | **Create a merge commit** | `release/*`, `hotfix/*` |

> Squash merge 는 **PR 제목이 커밋 제목, PR 본문이 커밋 본문**이 된다 (저장소 설정).
> 그래서 PR 제목도 커밋 메시지와 같은 형식이어야 하며, CI 가 검사한다.
> `Closes #n` 은 본문 마지막 줄에 두면 커밋 footer 로 들어가고, 머지 시 이슈가 닫힌다.

### 리뷰 규칙

| 대상 브랜치 | 필요 승인 수 |
|---|---|
| `develop` | 3명 |
| `main` | 5명 |

- 승인 후 새 커밋을 push 하면 승인이 초기화되어 다시 리뷰를 받아야 한다.
- 리뷰 코멘트(conversation)가 모두 resolve 되어야 머지할 수 있다.
- PR 은 가능한 작게 — 하나의 PR 에는 하나의 목적만 담는다.
- PR 본문 마지막 줄에 관련 이슈를 연결한다. (`Closes #12`)

### PR 올리기 전 체크리스트

- [ ] 최신 `develop` 을 반영했다 (`git pull --rebase origin develop`)
- [ ] 빌드/테스트가 로컬에서 통과한다
- [ ] clang-format 검사와 `run-clang-tidy -warnings-as-errors='*'` 가 통과한다 (0.2 장과 0.3 장. 형식은 pre-commit hook 이 먼저 막는다)
- [ ] PR 제목이 Conventional Commits 형식이다
- [ ] 불필요한 파일(빌드 산출물, 데이터셋 원본, 모델 가중치 등)이 포함되지 않았다

---

## 4. 문서 파일 이름

- 저장소 루트의 안내 파일만 대문자로 사용한다: `README.md`, `CONTRIBUTING.md`, `LICENSE` (GitHub 가 인식하는 이름)
- `docs/` 아래 문서는 소문자 snake_case 로 사용한다: `docs/spec/icd.md`, `docs/design/system_architecture.md`
- `docs/` 하위 폴더는 문서 성격으로 나눈다: `spec/` 통신 규약(ICD, 영상, 프로세스 사이), `design/` 설계(아키텍처, 클래스 다이어그램), `guide/` 설치와 운용 안내
- 근거: PX4, MAVSDK, OpenCV, PyTorch 는 소스 파일과 같은 snake_case 를 문서에도 사용한다. 하위 폴더의 대문자 `.md` 는 도구가 정한 이름(`SUMMARY.md`, `CHANGELOG_*.md`)뿐이다.

---

## 5. 예시

```bash
# 1) 기능 브랜치 생성
git switch develop
git pull origin develop
git switch -c feature/12-onboard-frame-capture

# 2) 작업 & 커밋
git add onboard/
git commit -m "feat(onboard): 카메라 프레임 캡처 태스크 추가"

# 3) push 후 GitHub 에서 develop 대상으로 PR 생성
git push -u origin feature/12-onboard-frame-capture
```

머지된 브랜치는 GitHub 에서 자동 삭제된다. 로컬에서도 정리한다.

```bash
git switch develop
git pull origin develop
git branch -d feature/12-onboard-frame-capture
```
