#!/usr/bin/env bash
# C++ 코드에서 기본 정수 타입 int, unsigned 의 사용을 검사한다 (CONTRIBUTING.md 0.2 "C++ 정수 타입").
# short, long, long long 은 clang-tidy 의 google-runtime-int 가 검사하므로 여기서는 검사하지 않는다.
# 사용한 곳이 있으면 그 줄을 출력하고 exit 1. CI 의 clang-format 작업이 실행한다.
#
# 제외
# - main 의 선언: C++ 표준이 반환 타입과 argc 를 int 로 정한다
# - 주석만 있는 줄
# - 생성 코드(*/generated/*, common/mavlink/generated)와 탐지 담당 코드(onboard/vision/python/native)
# - Pi ARGUS 반입본(onboard/argus_pi): 팀 정수 타입 규칙 적용 전 구현과 외부 라이브러리(vendor). 정리는 후속 작업
# 단어 경계는 앞뒤가 영문자, 숫자, _, - 가 아닌 경우로 본다 (문자열 "key-int-max" 등을 제외하기 위해)
# 검사 대상: 작업 트리의 파일. git 에 추가하지 않은 새 파일도 검사하고(--untracked), .gitignore 의
# 파일(빌드 결과 등)은 검사하지 않는다
set -euo pipefail

cd "$(dirname "$0")/.."

found="$(git grep --untracked -nP '(?<![\w-])(int|unsigned)(?![\w-])' -- \
    'onboard/*.cpp' 'onboard/*.hpp' 'onboard/*.h' 'gcs/*.cpp' 'gcs/*.hpp' 'gcs/*.h' \
    ':!*/generated/*' ':!common/mavlink/generated' ':!onboard/vision/python/native' ':!onboard/argus_pi' |
    grep -vE '^[^:]+:[0-9]+:[[:space:]]*//' |
    grep -vE '(^|[^[:alnum:]_])int main\(' || true)"

if [ -n "$found" ]; then
    echo "기본 정수 타입 int, unsigned 대신 <cstdint> 의 크기가 정해진 타입을 사용한다 (CONTRIBUTING.md 0.2):" >&2
    echo "$found" >&2
    exit 1
fi
