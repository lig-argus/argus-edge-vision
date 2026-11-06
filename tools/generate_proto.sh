#!/usr/bin/env bash
# onboard/libs/protocol/proto/argus_msg/*.proto 에서 Python 코드(*_pb2.py, *_pb2.pyi)를 생성한다.
# 결과(onboard/libs/protocol/python/argus_msg/)는 저장소에 커밋한다. CI 가 다시 생성해 커밋본과 비교한다.
# proto/ 를 import 기준 디렉터리로, python/ 을 출력 디렉터리로 주면 proto/argus_msg/a.proto 가
# python/argus_msg/a_pb2.py 가 되고, .proto 사이의 import "argus_msg/a.proto" 가 Python 에서
# from argus_msg import a_pb2 로 생성된다.
# C++ 코드는 커밋하지 않고 CMake 가 빌드 때 생성한다 (onboard/libs/protocol/CMakeLists.txt).
#
# protoc 는 RPi, CI 와 같은 3.21.12 를 쓴다 (Ubuntu 24.04 apt protobuf-compiler. macOS 설치는 README).
# 다른 버전으로 생성하면 파일 머리말과 런타임 요구 버전이 달라져 CI 의 비교가 실패한다.
#   PROTOC=$HOME/.local/protobuf-21/bin/protoc tools/generate_proto.sh
#
# - 생성 디렉터리의 *_pb2.py, *_pb2.pyi 를 지우고 다시 만든다 (지운 .proto 의 옛 파일이 남지 않게)
# - protoc 는 생성 파일에 "DO NOT EDIT" 표시를 넣으므로 따로 붙이지 않는다
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
protoc="${PROTOC:-protoc}"
proto_dir="$repo_root/onboard/libs/protocol/proto"
python_dir="$repo_root/onboard/libs/protocol/python"
output="$python_dir/argus_msg"
required_version="3.21.12"

version="$("$protoc" --version | awk '{print $2}')"
if [ "$version" != "$required_version" ]; then
    echo "protoc $required_version 이 필요하다 (지금 $version): PROTOC 환경 변수로 지정한다" >&2
    exit 1
fi

mkdir -p "$output"
rm -f "$output"/*_pb2.py "$output"/*_pb2.pyi
"$protoc" -I"$proto_dir" --python_out="$python_dir" --pyi_out="$python_dir" "$proto_dir"/argus_msg/*.proto
echo "생성: $output"
ls "$output"/*_pb2.py "$output"/*_pb2.pyi
