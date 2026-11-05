#!/usr/bin/env bash
# libplateau_native.so 를 만든다 (RPi, g++ 13, OpenMP). 저장소 CI 는 만들지 않는다.
# -ffp-contract=off: 곱셈과 덧셈을 합치지 않아 double 계산이 NumPy 와 같게 반올림된다.
set -Eeuo pipefail
dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$dir/build"
g++ -O3 -std=c++17 -fopenmp -ffp-contract=off -fPIC -shared \
    "$dir/plateau_native.cpp" -o "$dir/build/libplateau_native.so"
echo "$dir/build/libplateau_native.so"
