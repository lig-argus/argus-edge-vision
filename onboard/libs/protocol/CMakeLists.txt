# RPi 안 ZeroMQ 토픽 이름, RPC 이름, 기본 엔드포인트 상수와 메시지 정의(.proto)
# (docs/spec/process_interfaces.md 3장, 5장)
# - 헤더는 include/argus/protocol/ 에 두고 #include "argus/protocol/<파일>.hpp" 로 부른다. namespace 는 argus::protocol
# - 메시지 정의는 proto/argus_msg/<토픽 이름>.proto, package argus.msg. 다른 .proto 는 import "argus_msg/<이름>.proto"
#   로 부른다 (proto/ 가 import 기준 디렉터리. 이 구성이어야 Python 생성 코드의 import 경로가 패키지 argus_msg 와
#   맞는다). C++ 코드는 빌드 때 generated/ 에 생성하고 #include "argus_msg/<이름>.pb.h" 로 부른다 (namespace
#   argus::msg). 커밋하지 않는다. Python 코드는 tools/generate_proto.sh 로 python/argus_msg/ 에 생성해 커밋한다
#   (CI 가 재생성해 비교).
# - 다른 모듈에 의존하지 않는다. protobuf 런타임만 링크한다.
# 타깃은 셋이다.
# - argus::protocol_names: 이름과 기본값 상수 헤더(topics.hpp, zmq_endpoints.hpp)만 제공하는 INTERFACE
#   라이브러리. protobuf 를 링크하지 않는다. 메시지 정의를 쓰지 않는 모듈(통신 구현과 무관한 어댑터
#   등)은 이 타깃만 링크한다. zmq_endpoints.hpp 는 ZeroMQ 로 통신하는 실행 파일의 main() 만 쓴다.
# - argus::protocol: 메시지 정의의 생성 코드. argus::protocol_names 를 PUBLIC 으로 링크하므로 상수
#   헤더도 함께 쓸 수 있다.
# - argus::protocol_idl: Fast DDS 용 IDL 타입(idl/argus_msg/<토픽 이름>.idl, module argus::idl)의 생성 코드.
#   onboard/CMakeLists.txt 의 ARGUS_BUILD_BUS_DDS 가 ON 일 때만 빌드한다.
add_library(argus_protocol_names INTERFACE)
add_library(argus::protocol_names ALIAS argus_protocol_names)
target_include_directories(argus_protocol_names INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(argus_protocol STATIC)
add_library(argus::protocol ALIAS argus_protocol)
# 생성 소스(generated/*.pb.cc)는 clang-tidy 검사에서 제외한다: run-clang-tidy 에 파일 필터
# '^(?!.*/generated/)' 를 준다 (CONTRIBUTING.md 0장, ci.yml onboard 작업)
protobuf_generate(TARGET argus_protocol LANGUAGE cpp
                  PROTOS proto/argus_msg/common.proto
                         proto/argus_msg/control_mode_request.proto
                         proto/argus_msg/control_setpoint.proto
                         proto/argus_msg/detection.proto
                         proto/argus_msg/fc_attitude.proto
                         proto/argus_msg/fc_mode_event.proto
                         proto/argus_msg/fc_state.proto
                         proto/argus_msg/follow_command.proto
                         proto/argus_msg/follow_status.proto
                         proto/argus_msg/perception_ipc.proto
                         proto/argus_msg/perception_observations.proto
                         proto/argus_msg/tracking_status.proto
                         proto/argus_msg/tracking_tracks.proto
                         proto/argus_msg/vision_frame_gray8.proto
                         proto/argus_msg/vision_npu_input_jpeg.proto
                         proto/argus_msg/vision_overlay_jpeg.proto
                  IMPORT_DIRS proto
                  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_include_directories(argus_protocol PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 생성 코드는 우리 경고 설정(-Wall -Wextra -Wpedantic -Wshadow)과 clang-tidy 검사에서 제외한다
target_include_directories(argus_protocol SYSTEM PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(argus_protocol
                      PUBLIC argus::protocol_names protobuf::libprotobuf argus_onboard_warnings)

argus_add_test(protocol_test SOURCES test/protocol_test.cpp LIBS argus::protocol)
argus_add_test(protocol_proto_test SOURCES test/proto_test.cpp LIBS argus::protocol)

# IDL 타입의 C++ 코드는 fastddsgen 4.3.0(Fast DDS 3.6.2 와 짝인 버전)으로 idl/generated/argus_msg/ 에 생성해
# 커밋한다 (tools/generate_idl.sh). 빌드와 CI 에는 fastddsgen(Java)이 필요 없다.
# #include "argus_msg/<이름>PubSubTypes.hpp" 로 부른다.
if(ARGUS_BUILD_BUS_DDS)
    find_package(fastcdr 2 REQUIRED)
    find_package(fastdds 3 REQUIRED)
    set(argus_idl_names common control_mode_request control_setpoint detection fc_attitude fc_mode_event
                        fc_state follow_command follow_status perception_observations tracking_status
                        tracking_tracks vision_frame_gray8)
    set(argus_idl_sources "")
    foreach(name IN LISTS argus_idl_names)
        list(APPEND argus_idl_sources idl/generated/argus_msg/${name}PubSubTypes.cxx
                                      idl/generated/argus_msg/${name}TypeObjectSupport.cxx)
    endforeach()
    add_library(argus_protocol_idl STATIC ${argus_idl_sources})
    add_library(argus::protocol_idl ALIAS argus_protocol_idl)
    # 생성 코드는 우리 경고 설정과 clang-tidy 검사에서 제외한다
    target_include_directories(argus_protocol_idl SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/idl/generated)
    target_link_libraries(argus_protocol_idl PUBLIC fastdds fastcdr)
endif()
