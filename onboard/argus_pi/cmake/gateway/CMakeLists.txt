# C++ FC 게이트웨이 (Python tools/real_fc 게이트웨이 단계별 이식).
# 기존 follow_b/perception 빌드와 따로 구성한다. 이 구성은 장치·FC를 열지 않는다.
cmake_minimum_required(VERSION 3.16)
project(argus_gateway LANGUAGES CXX)
get_filename_component(ARGUS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(nlohmann_json QUIET)
find_package(LibLZMA REQUIRED)
find_package(ZLIB REQUIRED)

# 1단계: 설정·송신 잠금·FC 이벤트
add_library(argus_gateway_core STATIC
    ${ARGUS_ROOT}/src/argus/gateway/common/python_compat.cpp
    ${ARGUS_ROOT}/src/argus/gateway/common/sha256.cpp
    ${ARGUS_ROOT}/src/argus/gateway/settings/deployment_settings.cpp
    ${ARGUS_ROOT}/src/argus/gateway/ownership/command_owner.cpp
    ${ARGUS_ROOT}/src/argus/gateway/events/event_parser.cpp
    ${ARGUS_ROOT}/src/argus/gateway/events/event_log.cpp
    # 2단계: MAVLink 코덱·링크, FC 관제 연결부(vehicle.py)
    ${ARGUS_ROOT}/src/argus/gateway/mavlink/mavlink_codec.cpp
    ${ARGUS_ROOT}/src/argus/gateway/mavlink/mavlink_link.cpp
    ${ARGUS_ROOT}/src/argus/gateway/vehicle/flight_link.cpp
    ${ARGUS_ROOT}/src/argus/gateway/vehicle/mag_interference.cpp
    ${ARGUS_ROOT}/src/argus/gateway/vehicle/rc_reclaim.cpp
    # 4단계: msgpack, CV 관측·진단 기록, 추종기 요청
    ${ARGUS_ROOT}/src/argus/gateway/common/msgpack_codec.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/cv_observation.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/cv_journal.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/follow_proxy.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/manual_yaw.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/follow_owner.cpp
    ${ARGUS_ROOT}/src/argus/gateway/follow/zmq_transport.cpp
    # 5단계: 메인 GUI 연결부 (대상 신원, 추종 요청, 추종 설정)
    ${ARGUS_ROOT}/src/argus/gateway/bridge/track_identity.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/track_adapter.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/follow_preferences.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/gcs_mavlink.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/profile_coordinator.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/main_gcs_bridge.cpp
    ${ARGUS_ROOT}/src/argus/gateway/bridge/udp_sockets.cpp
    # 6단계: 추종기·영상 준비, 정비, HTTP API
    ${ARGUS_ROOT}/src/argus/gateway/service/process_supervisor.cpp
    ${ARGUS_ROOT}/src/argus/gateway/service/preview_supervisor.cpp
    ${ARGUS_ROOT}/src/argus/gateway/service/follow_service.cpp
    ${ARGUS_ROOT}/src/argus/gateway/service/maintenance.cpp
    ${ARGUS_ROOT}/src/argus/gateway/http/http_server.cpp
    ${ARGUS_ROOT}/src/argus/gateway/http/gateway_api.cpp)
find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(ZMQ_LIBRARY zmq REQUIRED)
include(${ARGUS_ROOT}/cmake/mavlink.cmake)
find_package(Threads REQUIRED)
target_include_directories(argus_gateway_core PUBLIC ${ARGUS_ROOT}/src)
target_link_libraries(argus_gateway_core PRIVATE argus_mavlink)
target_link_libraries(argus_gateway_core PUBLIC Threads::Threads)
target_include_directories(argus_gateway_core SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
target_link_libraries(argus_gateway_core PUBLIC ${ZMQ_LIBRARY})
target_compile_options(argus_gateway_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PUBLIC LibLZMA::LibLZMA ZLIB::ZLIB)
if(nlohmann_json_FOUND)
    target_link_libraries(argus_gateway_core PUBLIC nlohmann_json::nlohmann_json)
endif()
# 2026-11-25 batch 2: tracking/tracks 는 팀 규약 [이름][protobuf] (6375). 본문 변환(team_wire tracks_body)은
# 따로 묶어 PRIVATE 로 링크한다. 변환 코드의 include 경로(vendor/vmy nlohmann)가 게이트웨이 번역 단위에
# 들어가지 않게 한다 (src/argus/gateway/follow/include/tracks_protobuf.hpp)
include(${ARGUS_ROOT}/cmake/argus_bus.cmake)
add_library(argus_gateway_tracks_pb STATIC ${ARGUS_ROOT}/src/argus/gateway/follow/tracks_protobuf.cpp)
target_link_libraries(argus_gateway_tracks_pb PRIVATE argus_team_wire)
target_compile_options(argus_gateway_tracks_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_tracks_pb)
# 2026-11-25 batch 4: 영상 준비 확인은 팀 vision 6374 [이름][protobuf][binary] 를 받는다. 본문 변환(team_wire
# vision_body)도 같은 방식으로 PRIVATE 로 묶는다 (src/argus/gateway/service/include/vision_protobuf.hpp)
add_library(argus_gateway_vision_pb STATIC ${ARGUS_ROOT}/src/argus/gateway/service/vision_protobuf.cpp)
target_link_libraries(argus_gateway_vision_pb PRIVATE argus_team_wire)
target_compile_options(argus_gateway_vision_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_vision_pb)
# 2026-11-25 batch 6a: FC 텔레메트리 6370 은 팀 [이름][protobuf] 로 보내고 (ZmqFollowTransport), 정비
# 지상 확인이 같은 본문을 받는다. 변환(team_wire)은 같은 방식으로 PRIVATE 로 묶는다 (fc_protobuf.hpp)
add_library(argus_gateway_fc_pb STATIC ${ARGUS_ROOT}/src/argus/gateway/follow/fc_protobuf.cpp)
target_link_libraries(argus_gateway_fc_pb PRIVATE argus_team_wire Threads::Threads)
target_compile_options(argus_gateway_fc_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_fc_pb)
# 2026-11-25 batch 6b: 추종 제어의 6372 (control/setpoint, follow/status) 는 팀 [이름][protobuf] 로 받는다
# (ZmqFollowTransport::command_receive). 변환(team_wire)은 같은 방식으로 PRIVATE 로 묶는다 (control_protobuf.hpp)
add_library(argus_gateway_control_pb STATIC ${ARGUS_ROOT}/src/argus/gateway/follow/control_protobuf.cpp)
target_link_libraries(argus_gateway_control_pb PRIVATE argus_team_wire)
target_compile_options(argus_gateway_control_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_control_pb)
# 2026-11-25 batch 7: 6371 (control/mode_request 요청-응답) 은 팀 [이름][protobuf] 로 받고 답한다
# (ZmqFollowTransport::rpc_receive, rpc_send). 같은 방식으로 PRIVATE 로 묶는다 (mode_rpc_protobuf.hpp)
add_library(argus_gateway_mode_rpc_pb STATIC ${ARGUS_ROOT}/src/argus/gateway/follow/mode_rpc_protobuf.cpp)
target_link_libraries(argus_gateway_mode_rpc_pb PRIVATE argus_team_wire)
target_compile_options(argus_gateway_mode_rpc_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_mode_rpc_pb)
# 2026-11-25 batch 8: 운용자 요청 6373 (follow/command 요청-응답) 은 팀 [이름][protobuf] 로 보내고 받는다
# (FollowProxy::exchange). 같은 방식으로 PRIVATE 로 묶는다 (follow_command_protobuf.hpp)
add_library(argus_gateway_follow_command_pb STATIC
    ${ARGUS_ROOT}/src/argus/gateway/follow/follow_command_protobuf.cpp)
target_link_libraries(argus_gateway_follow_command_pb PRIVATE argus_team_wire)
target_compile_options(argus_gateway_follow_command_pb PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(argus_gateway_core PRIVATE argus_gateway_follow_command_pb)

option(ARGUS_GATEWAY_TESTS "게이트웨이 시험 빌드" ON)
if(ARGUS_GATEWAY_TESTS)
    enable_testing()
    add_executable(GatewayStage1Test ${ARGUS_ROOT}/tests/gateway/gateway_stage1_test.cpp)
    target_link_libraries(GatewayStage1Test PRIVATE argus_gateway_core)
    target_compile_options(GatewayStage1Test PRIVATE -Wall -Wextra)
    add_test(NAME GatewayStage1Test COMMAND GatewayStage1Test)
    # Python 원본과 같은 입력을 넣어 출력이 같은지 보는 비교 도구 (tests/gateway/differential)
    add_executable(argus_gateway_probe ${ARGUS_ROOT}/tests/gateway/gateway_probe.cpp)
    target_link_libraries(argus_gateway_probe PRIVATE argus_gateway_core)
    # 2단계: 모의 FC 바이트를 넣고 송신 바이트·상태를 내는 비교 도구, 코덱 비교 도구
    add_executable(argus_gateway_vehicle_probe ${ARGUS_ROOT}/tests/gateway/gateway_vehicle_probe.cpp)
    target_link_libraries(argus_gateway_vehicle_probe PRIVATE argus_gateway_core)
    add_executable(GatewayStage2Test ${ARGUS_ROOT}/tests/gateway/gateway_stage2_test.cpp)
    target_link_libraries(GatewayStage2Test PRIVATE argus_gateway_core)
    add_test(NAME GatewayStage2Test COMMAND GatewayStage2Test)
    add_executable(GatewayStage3Test ${ARGUS_ROOT}/tests/gateway/gateway_stage3_test.cpp)
    target_link_libraries(GatewayStage3Test PRIVATE argus_gateway_core argus_mavlink)
    add_test(NAME GatewayStage3Test COMMAND GatewayStage3Test)
    add_executable(GatewayWireConversionTest ${ARGUS_ROOT}/tests/gateway/gateway_wire_conversion_test.cpp)
    target_link_libraries(GatewayWireConversionTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayWireConversionTest COMMAND GatewayWireConversionTest)
    # 2026-11-16 FC setpoint 이벤트 송신 시각 규칙
    add_executable(GatewaySetpointEventTest ${ARGUS_ROOT}/tests/gateway/gateway_setpoint_event_test.cpp)
    target_link_libraries(GatewaySetpointEventTest PRIVATE argus_gateway_core)
    add_test(NAME GatewaySetpointEventTest COMMAND GatewaySetpointEventTest)
    add_executable(GatewayStage4Test ${ARGUS_ROOT}/tests/gateway/gateway_stage4_test.cpp)
    target_include_directories(GatewayStage4Test SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    # 2026-11-25 batch 7: 6371 요청, 응답 본문은 시험 라이브러리 (아래 argus_gateway_control_fixture) 로
    target_include_directories(GatewayStage4Test PRIVATE ${ARGUS_ROOT})
    target_link_libraries(GatewayStage4Test PRIVATE argus_gateway_core argus_gateway_fc_fixture
        argus_gateway_control_fixture)
    add_test(NAME GatewayStage4Test COMMAND GatewayStage4Test)
    add_executable(GatewayStage5Test ${ARGUS_ROOT}/tests/gateway/gateway_stage5_test.cpp)
    target_include_directories(GatewayStage5Test SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    target_link_libraries(GatewayStage5Test PRIVATE argus_gateway_core)
    add_test(NAME GatewayStage5Test COMMAND GatewayStage5Test)
    add_executable(GatewayStage6Test ${ARGUS_ROOT}/tests/gateway/gateway_stage6_test.cpp)
    target_link_libraries(GatewayStage6Test PRIVATE argus_gateway_core)
    add_test(NAME GatewayStage6Test COMMAND GatewayStage6Test)
    # 2026-11-12 실외 장착각 모드(30°/45°) 선택·경로
    add_executable(GatewayOutdoorProfileTest ${ARGUS_ROOT}/tests/gateway/gateway_outdoor_profile_test.cpp)
    target_link_libraries(GatewayOutdoorProfileTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayOutdoorProfileTest COMMAND GatewayOutdoorProfileTest)
    add_executable(GatewayScanTargetTest ${ARGUS_ROOT}/tests/gateway/gateway_scan_target_test.cpp)
    target_link_libraries(GatewayScanTargetTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayScanTargetTest COMMAND GatewayScanTargetTest)
    # 2026-11-13 탐색 회전 가감속 램프
    add_executable(GatewayScanRampTest ${ARGUS_ROOT}/tests/gateway/gateway_scan_ramp_test.cpp)
    target_link_libraries(GatewayScanRampTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayScanRampTest COMMAND GatewayScanRampTest)
    # 2026-11-18 실내 데모 모드: TRACK_START 회전 모드, 이륙/고도 변경 1.0~2.0 m
    add_executable(GatewayDemoModeTest ${ARGUS_ROOT}/tests/gateway/gateway_demo_mode_test.cpp)
    target_link_libraries(GatewayDemoModeTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayDemoModeTest COMMAND GatewayDemoModeTest)
    # 2026-11-18 실내 데모 위치 고정: LOCAL_NED 위치 유지 + yaw rate, 비데모는 BODY_NED 속도 그대로
    add_executable(GatewayDemoAnchorTest ${ARGUS_ROOT}/tests/gateway/gateway_demo_anchor_test.cpp)
    target_link_libraries(GatewayDemoAnchorTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayDemoAnchorTest COMMAND GatewayDemoAnchorTest)
    # 2026-11-18 이륙 조건 보강: FC 강한 자기장 간섭 경고로 ARM, 이륙 거부
    add_executable(GatewayMagGateTest ${ARGUS_ROOT}/tests/gateway/gateway_mag_gate_test.cpp)
    target_link_libraries(GatewayMagGateTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayMagGateTest COMMAND GatewayMagGateTest)
    # 2026-11-23 영상 준비 판정 (JPEG 미리보기, 흑백 영상)
    add_executable(GatewayPreviewReadyTest ${ARGUS_ROOT}/tests/gateway/gateway_preview_ready_test.cpp)
    target_link_libraries(GatewayPreviewReadyTest PRIVATE argus_gateway_core)
    add_test(NAME GatewayPreviewReadyTest COMMAND GatewayPreviewReadyTest)
    # 2026-11-25 batch 2 리뷰: tracking/tracks 받기(tracks_msgpack)를 실제 team_wire 본문으로 본다.
    # 본문을 만드는 쪽은 argus_team_wire 를 PRIVATE 로 링크한 시험 라이브러리에 둔다 (nlohmann 경로 분리)
    add_library(argus_gateway_tracks_fixture STATIC ${ARGUS_ROOT}/tests/gateway/gateway_tracks_fixture.cpp)
    target_include_directories(argus_gateway_tracks_fixture PRIVATE ${ARGUS_ROOT})
    target_compile_definitions(argus_gateway_tracks_fixture PRIVATE
        ARGUS_TEAM_WIRE_FIXTURES="${ARGUS_ROOT}/tests/team_wire/fixtures")
    target_compile_options(argus_gateway_tracks_fixture PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_link_libraries(argus_gateway_tracks_fixture PRIVATE argus_team_wire)
    add_executable(GatewayTracksProtobufTest ${ARGUS_ROOT}/tests/gateway/gateway_tracks_protobuf_test.cpp)
    target_include_directories(GatewayTracksProtobufTest PRIVATE ${ARGUS_ROOT})
    target_link_libraries(GatewayTracksProtobufTest PRIVATE argus_gateway_core argus_gateway_tracks_pb
        argus_gateway_tracks_fixture)
    add_test(NAME GatewayTracksProtobufTest COMMAND GatewayTracksProtobufTest)
    # 2026-11-25 batch 4: 영상 준비 확인의 6374 받기(vision_msgpack)를 실제 team_wire 본문으로 본다
    add_library(argus_gateway_vision_fixture STATIC ${ARGUS_ROOT}/tests/gateway/gateway_vision_fixture.cpp)
    target_include_directories(argus_gateway_vision_fixture PRIVATE ${ARGUS_ROOT})
    target_compile_options(argus_gateway_vision_fixture PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_link_libraries(argus_gateway_vision_fixture PRIVATE argus_team_wire)
    add_executable(GatewayVisionProtobufTest ${ARGUS_ROOT}/tests/gateway/gateway_vision_protobuf_test.cpp)
    target_include_directories(GatewayVisionProtobufTest PRIVATE ${ARGUS_ROOT})
    target_link_libraries(GatewayVisionProtobufTest PRIVATE argus_gateway_core argus_gateway_vision_pb
        argus_gateway_vision_fixture)
    add_test(NAME GatewayVisionProtobufTest COMMAND GatewayVisionProtobufTest ${ARGUS_ROOT})
    # 2026-11-25 batch 6a: FC 텔레메트리 6370 보내기 (ZmqFollowTransport) 와 정비 지상 확인의 받기를 실제
    # team_wire 본문으로 본다. 본문 해석과 만들기는 argus_team_wire 를 PRIVATE 로 링크한 시험 라이브러리
    add_library(argus_gateway_fc_fixture STATIC ${ARGUS_ROOT}/tests/gateway/gateway_fc_fixture.cpp)
    target_include_directories(argus_gateway_fc_fixture PRIVATE ${ARGUS_ROOT})
    target_compile_options(argus_gateway_fc_fixture PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_link_libraries(argus_gateway_fc_fixture PRIVATE argus_team_wire)
    add_executable(GatewayFcProtobufTest ${ARGUS_ROOT}/tests/gateway/gateway_fc_protobuf_test.cpp)
    target_include_directories(GatewayFcProtobufTest PRIVATE ${ARGUS_ROOT})
    target_include_directories(GatewayFcProtobufTest SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    target_compile_options(GatewayFcProtobufTest PRIVATE -Wall -Wextra -UNDEBUG)
    target_link_libraries(GatewayFcProtobufTest PRIVATE argus_gateway_core argus_gateway_fc_fixture)
    add_test(NAME GatewayFcProtobufTest COMMAND GatewayFcProtobufTest ${ARGUS_ROOT})
    set_tests_properties(GatewayFcProtobufTest PROPERTIES TIMEOUT 60)
    # 2026-11-25 batch 6b: 6372 받기 (ZmqFollowTransport::command_receive) 를 실제 team_wire 본문으로 본다.
    # 32 행 넘는 follow/status 본문 만들기는 argus_team_wire 를 PRIVATE 로 링크한 시험 라이브러리
    add_library(argus_gateway_control_fixture STATIC ${ARGUS_ROOT}/tests/gateway/gateway_control_fixture.cpp)
    target_include_directories(argus_gateway_control_fixture PRIVATE ${ARGUS_ROOT})
    target_compile_options(argus_gateway_control_fixture PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_link_libraries(argus_gateway_control_fixture PRIVATE argus_team_wire)
    add_executable(GatewayControlProtobufTest ${ARGUS_ROOT}/tests/gateway/gateway_control_protobuf_test.cpp)
    target_include_directories(GatewayControlProtobufTest PRIVATE ${ARGUS_ROOT})
    target_include_directories(GatewayControlProtobufTest SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    target_compile_options(GatewayControlProtobufTest PRIVATE -Wall -Wextra -UNDEBUG)
    target_link_libraries(GatewayControlProtobufTest PRIVATE argus_gateway_core argus_gateway_fc_fixture
        argus_gateway_control_fixture)
    add_test(NAME GatewayControlProtobufTest COMMAND GatewayControlProtobufTest ${ARGUS_ROOT})
    set_tests_properties(GatewayControlProtobufTest PROPERTIES TIMEOUT 60)
    # 2026-11-25 batch 7: 6371 요청-응답 (ZmqFollowTransport::rpc_receive, rpc_send 와 실제 FollowBOwner)
    add_executable(GatewayModeRpcProtobufTest ${ARGUS_ROOT}/tests/gateway/gateway_mode_rpc_protobuf_test.cpp)
    target_include_directories(GatewayModeRpcProtobufTest PRIVATE ${ARGUS_ROOT})
    target_include_directories(GatewayModeRpcProtobufTest SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    target_compile_options(GatewayModeRpcProtobufTest PRIVATE -Wall -Wextra -UNDEBUG)
    target_link_libraries(GatewayModeRpcProtobufTest PRIVATE argus_gateway_core argus_gateway_fc_fixture
        argus_gateway_control_fixture)
    add_test(NAME GatewayModeRpcProtobufTest COMMAND GatewayModeRpcProtobufTest)
    set_tests_properties(GatewayModeRpcProtobufTest PROPERTIES TIMEOUT 60)
    # 2026-11-25 batch 8: 6373 운용자 요청-응답 (FollowProxy 의 팀 [follow/command][protobuf])
    add_executable(GatewayFollowCommandProtobufTest
        ${ARGUS_ROOT}/tests/gateway/gateway_follow_command_protobuf_test.cpp)
    target_include_directories(GatewayFollowCommandProtobufTest PRIVATE ${ARGUS_ROOT})
    target_include_directories(GatewayFollowCommandProtobufTest SYSTEM PRIVATE ${ZMQ_INCLUDE_DIR})
    target_compile_options(GatewayFollowCommandProtobufTest PRIVATE -Wall -Wextra -UNDEBUG)
    target_link_libraries(GatewayFollowCommandProtobufTest PRIVATE argus_gateway_core
        argus_gateway_control_fixture)
    add_test(NAME GatewayFollowCommandProtobufTest COMMAND GatewayFollowCommandProtobufTest)
    set_tests_properties(GatewayFollowCommandProtobufTest PROPERTIES TIMEOUT 30)
    add_executable(argus_gateway_follow_probe ${ARGUS_ROOT}/tests/gateway/gateway_follow_probe.cpp)
    target_link_libraries(argus_gateway_follow_probe PRIVATE argus_gateway_core)
    # 6단계 follow_service 비교 시험의 가짜 추적기·추종기 (포트에서 듣기만 한다)
    add_executable(argus_gateway_fake_service ${ARGUS_ROOT}/tests/gateway/gateway_fake_service.cpp)
endif()

# C++ FC 게이트웨이 실행 파일 (원본 python -m tools.real_fc.gateway)
add_executable(argus_fc_gateway ${ARGUS_ROOT}/src/argus/gateway/app/gateway_main.cpp)
target_link_libraries(argus_fc_gateway PRIVATE argus_gateway_core)

