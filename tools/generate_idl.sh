#!/usr/bin/env bash
# onboard/libs/protocol/idl/argus_msg/*.idl 에서 Fast DDS 용 C++ 코드를 생성한다.
# 결과(onboard/libs/protocol/idl/generated/argus_msg/)는 저장소에 커밋한다. 빌드와 CI 에는
# fastddsgen(Java)이 필요 없다. 사용하는 쪽은 onboard/libs/protocol/idl/generated/ 를 include 경로로 두고
# #include "argus_msg/<이름>PubSubTypes.hpp" 로 부른다.
#
# fastddsgen 은 Fast DDS 3.6.2 와 짝인 4.3.0 을 사용한다. 다른 버전으로 생성하면 생성 코드가 링크하는
# Fast DDS API 와 파일 머리말이 달라진다.
#   FASTDDSGEN=$HOME/.local/fastdds/bin/fastddsgen tools/generate_idl.sh
#
# - IDL 사이의 #include 를 처리하도록 전처리기(cpp)를 켠다. IDL 이 있는 디렉터리를 include 경로로 준다
# - fastddsgen 은 전처리 결과를 임시 디렉터리에 <파일>.idl.cc 로 기록하고 읽는다. 이전 실행의 파일을
#   읽지 않도록 실행마다 새 임시 디렉터리를 만들고 끝나면 지운다
# - 생성 디렉터리의 생성 파일을 지우고 다시 만든다 (지운 .idl 의 옛 파일이 남지 않게)
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
fastddsgen="${FASTDDSGEN:-fastddsgen}"
idl_dir="$repo_root/onboard/libs/protocol/idl/argus_msg"
output="$repo_root/onboard/libs/protocol/idl/generated/argus_msg"
required_version="4.3.0"

version="$("$fastddsgen" -version | awk '{print $3}')"
if [ "$version" != "$required_version" ]; then
    echo "fastddsgen $required_version 이 필요하다 (현재 $version): FASTDDSGEN 환경 변수로 지정한다" >&2
    exit 1
fi

temp_dir="$(mktemp -d)"
trap 'rm -rf "$temp_dir"' EXIT

mkdir -p "$output"
rm -f "$output"/*.hpp "$output"/*.ipp "$output"/*.cxx
cd "$idl_dir"
"$fastddsgen" -replace -t "$temp_dir" -d "$output" -I "$idl_dir" *.idl
echo "생성: $output"
ls "$output"
