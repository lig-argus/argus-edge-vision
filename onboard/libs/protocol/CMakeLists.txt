# RPi 안 ZeroMQ 토픽 이름, RPC 이름, 기본 엔드포인트 상수와 메시지 정의(.proto)
# (docs/spec/process_interfaces.md 3장, 5장)
# - 헤더는 include/argus/protocol/ 에 두고 #include "argus/protocol/<파일>.hpp" 로 부른다. namespace 는 argus::protocol
# - 메시지 정의는 proto/<토픽 이름>.proto, package argus.msg. C++ 코드는 빌드 때 generated/ 에 생성하고
#   #include "<이름>.pb.h" 로 부른다 (namespace argus::msg). 커밋하지 않는다.
#   Python 코드는 tools/generate_proto.sh 로 python/argus_msg/ 에 생성해 커밋한다 (CI 가 재생성해 비교).
# - 다른 모듈에 의존하지 않는다. protobuf 런타임만 링크한다.
add_library(argus_protocol STATIC)
add_library(argus::protocol ALIAS argus_protocol)
# 생성 소스(generated/*.pb.cc)는 clang-tidy 검사에서 제외한다: run-clang-tidy 에 파일 필터
# '^(?!.*/generated/)' 를 준다 (CONTRIBUTING.md 0장, ci.yml onboard 작업)
protobuf_generate(TARGET argus_protocol LANGUAGE cpp
                  PROTOS proto/control_setpoint.proto
                  IMPORT_DIRS proto
                  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_include_directories(argus_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 생성 코드는 우리 경고 설정(-Wall -Wextra -Wpedantic -Wshadow)과 clang-tidy 검사에서 제외한다
target_include_directories(argus_protocol SYSTEM PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(argus_protocol PUBLIC protobuf::libprotobuf argus_onboard_warnings)

argus_add_test(protocol_test SOURCES test/protocol_test.cpp LIBS argus::protocol)
argus_add_test(protocol_proto_test SOURCES test/proto_test.cpp LIBS argus::protocol)
