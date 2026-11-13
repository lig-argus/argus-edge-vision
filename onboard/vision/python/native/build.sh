#!/usr/bin/env bash
# libplateau_native.so 를 만든다: onboard CMake 의 argus_plateau_native 타깃만 빌드한다
# (onboard/vision/CMakeLists.txt). 출력은 build/onboard/vision/libplateau_native.so 이고,
# argus_vision/plateau_native.py 의 기본 경로와 같다.
# build/onboard 가 아직 없으면 README 와 같이 설정한다 (Ninja 가 있으면 Ninja, Release).
# 의존성은 onboard 와 같다 (onboard/CMakeLists.txt). OpenMP 는 Ubuntu g++ 에 포함,
# macOS 는 brew install libomp 뒤 -DOpenMP_ROOT="$(brew --prefix libomp)" 로 설정한다
set -Eeuo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
build="$repo/build/onboard"
if [ ! -f "$build/CMakeCache.txt" ]; then
    generator=()
    if command -v ninja >/dev/null; then
        generator=(-G Ninja)
    fi
    cmake -S "$repo/onboard" -B "$build" "${generator[@]}" -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$build" --target argus_plateau_native
echo "$build/vision/libplateau_native.so"
