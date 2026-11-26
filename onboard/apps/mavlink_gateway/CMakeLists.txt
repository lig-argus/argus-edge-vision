# mavlink_gateway 프로세스: FC 통신, 통제기 통신 (docs/design/uml/class_mavlink_gateway.md). 통신 담당
# 실행 파일 이름: argus_mavlink_gateway (main.cpp 로 add_executable). 그 밖의 소스는 라이브러리 argus_mavlink_gateway_core(STATIC)에
# 추가하고 실행 파일과 테스트가 링크한다 (CONTRIBUTING.md 0.4). 모두 argus_onboard_warnings 를 링크한다.
# 헤더는 include/argus/mavlink_gateway/, namespace 는 argus::mavlink_gateway

# mavlink_gateway 안 클래스들이 쓰는 인터페이스 헤더 (IMavlinkConnection, IMavlinkCommandHandler)
add_library(argus_mavlink_gateway_interfaces INTERFACE)
add_library(argus::mavlink_gateway_interfaces ALIAS argus_mavlink_gateway_interfaces)
target_include_directories(argus_mavlink_gateway_interfaces
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_mavlink_gateway_interfaces
                      INTERFACE argus::core argus_mavlink argus_onboard_warnings)

# FakeMavlinkConnection 은 mavlink_gateway 테스트만 쓰므로 test/ 에 둔다
argus_add_test(mavlink_gateway_fake_mavlink_connection_test
               SOURCES test/fake_mavlink_connection_test.cpp
               LIBS argus::mavlink_gateway_interfaces argus::core_testing)
argus_add_test(mavlink_gateway_command_responder_test SOURCES test/command_responder_test.cpp
               LIBS argus::mavlink_gateway_interfaces)

# main.cpp 를 제외한 mavlink_gateway 의 소스. 실행 파일과 테스트가 링크한다
add_library(argus_mavlink_gateway_core STATIC
    src/argus_sensor_status_reporter.cpp
    src/argus_status_policy.cpp
    src/argus_status_reporter.cpp
    src/control_authority.cpp
    src/control_authority_policy.cpp
    src/fc_command_policy.cpp
    src/fc_control.cpp
    src/fc_gateway.cpp
    src/fc_link_tracker.cpp
    src/fc_telemetry_relay.cpp
    src/fc_telemetry_receiver.cpp
    src/follow_activity.cpp
    src/gcs_fc_forwarder.cpp
    src/gcs_forward_policy.cpp
    src/gcs_link_monitor.cpp
    src/gcs_message_encoding.cpp
    src/gcs_registry.cpp
    src/heartbeat_sender.cpp
    src/mavlink_command_receiver.cpp
    src/mode_command_encoding.cpp
    src/px4_flight_mode.cpp
    src/request_message_handler.cpp
    src/request_operator_control_handler.cpp
    src/sensor_status_policy.cpp
    src/setpoint_encoding.cpp
    src/sysfs_cpu_temperature.cpp
    src/time_sync.cpp
    src/udp_gcs_link.cpp
    src/udp_mavlink_connection.cpp
    src/udp_socket.cpp
    src/video_stream_information.cpp)
add_library(argus::mavlink_gateway_core ALIAS argus_mavlink_gateway_core)
# FcGateway 가 bus 인터페이스와 vehicle_api 의 bus 어댑터를 사용한다. VIDEO_STREAM_INFORMATION 은
# video_api 의 조회 인터페이스로 값을 읽는다. 통신 구현(bus_zmq)은 링크하지 않는다 (실행 파일만 링크)
target_link_libraries(argus_mavlink_gateway_core
    PUBLIC argus::mavlink_gateway_interfaces argus::vehicle_api argus::bus argus::vehicle_api_bus
           argus::video_api argus::perception_api
           argus_onboard_warnings)

# 통신 구현(DDS 또는 ZeroMQ, UDP)과 실제 시계는 main.cpp 에서만 생성한다
# (docs/design/uml/class_mavlink_gateway.md 4절)
add_executable(argus_mavlink_gateway src/main.cpp)
# DDS 빌드에서만 DDS 구현을 링크한다. main.cpp 는 ARGUS_HAS_BUS_DDS 로 --bus dds 를 켠다.
# vehicle_api_bus_zmq 보다 먼저 적는다. include 경로가 적은 순서대로 붙는데, cppzmq 의 include 경로(macOS
# Homebrew 는 /opt/homebrew/include)에 다른 버전의 Fast DDS 헤더가 있으면 그 헤더가 먼저 잡힌다
if(ARGUS_BUILD_BUS_DDS)
    target_link_libraries(argus_mavlink_gateway
                          PRIVATE argus::vehicle_api_bus_dds argus::video_api_bus_dds
                                  argus::perception_api_bus_dds)
    target_compile_definitions(argus_mavlink_gateway PRIVATE ARGUS_HAS_BUS_DDS)
endif()
target_link_libraries(argus_mavlink_gateway
    PRIVATE argus::mavlink_gateway_core argus::vehicle_api_bus_zmq argus::video_api_bus_zmq
            argus::perception_api_bus argus::perception_api_bus_zmq argus::protocol_names
            argus_onboard_warnings)

argus_add_test(mavlink_gateway_health_state_test SOURCES test/health_state_test.cpp
               LIBS argus::mavlink_gateway_interfaces)
argus_add_test(mavlink_gateway_px4_flight_mode_test SOURCES test/px4_flight_mode_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_mavlink_handler_registry_test SOURCES test/mavlink_handler_registry_test.cpp
               LIBS argus::mavlink_gateway_interfaces)
# UdpMavlinkConnection 테스트는 loopback UDP 소켓을 실제로 연다
argus_add_test(mavlink_gateway_udp_mavlink_connection_test SOURCES test/udp_mavlink_connection_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_time_sync_test SOURCES test/time_sync_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_fc_link_tracker_test SOURCES test/fc_link_tracker_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_fc_telemetry_receiver_test SOURCES test/fc_telemetry_receiver_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_fc_command_policy_test SOURCES test/fc_command_policy_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_setpoint_inbox_test SOURCES test/setpoint_inbox_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_fc_control_test SOURCES test/fc_control_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
# FC 명령 송신을 실제 구현으로 연결한 시나리오 테스트
argus_add_test(mavlink_gateway_fc_command_scenario_test SOURCES test/fc_command_scenario_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_fc_state_view_test SOURCES test/fc_state_view_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
# FC 쪽 객체를 FakeBus 와 연결한 테스트. 추종 제어의 요청은 실제 bus 어댑터로 보낸다
argus_add_test(mavlink_gateway_fc_gateway_test SOURCES test/fc_gateway_test.cpp
               LIBS argus::mavlink_gateway_core argus::bus_testing argus::core_testing
                    argus::protocol_names)

# 통제기 쪽 (통제기 연결, 제어권, 명령 수신, 두절 감시). FakeGcsLink 는 test/ 에 둔다
# UdpGcsLink 테스트는 loopback UDP 소켓을 실제로 연다
argus_add_test(mavlink_gateway_udp_gcs_link_test SOURCES test/udp_gcs_link_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_gcs_registry_test SOURCES test/gcs_registry_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_control_authority_policy_test
               SOURCES test/control_authority_policy_test.cpp LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_control_authority_test SOURCES test/control_authority_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing argus::vehicle_api_testing)
argus_add_test(mavlink_gateway_mavlink_command_receiver_test
               SOURCES test/mavlink_command_receiver_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_request_message_handler_test
               SOURCES test/request_message_handler_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_video_stream_information_test
               SOURCES test/video_stream_information_test.cpp LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_request_operator_control_handler_test
               SOURCES test/request_operator_control_handler_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_gcs_link_monitor_test SOURCES test/gcs_link_monitor_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing argus::vehicle_api_testing)

# C 단계: ARGUS_STATUS 송신과 추종 제어 두절 처리
argus_add_test(mavlink_gateway_argus_status_policy_test SOURCES test/argus_status_policy_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_argus_status_reporter_test
               SOURCES test/argus_status_reporter_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing argus::vehicle_api_testing)
argus_add_test(mavlink_gateway_sysfs_cpu_temperature_test
               SOURCES test/sysfs_cpu_temperature_test.cpp LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_sensor_status_policy_test SOURCES test/sensor_status_policy_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_argus_sensor_status_reporter_test
               SOURCES test/argus_sensor_status_reporter_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
argus_add_test(mavlink_gateway_heartbeat_sender_test SOURCES test/heartbeat_sender_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)

# E 단계: 통제기 메시지의 FC 전달과 FC telemetry 전달
argus_add_test(mavlink_gateway_gcs_forward_policy_test SOURCES test/gcs_forward_policy_test.cpp
               LIBS argus::mavlink_gateway_core)
argus_add_test(mavlink_gateway_gcs_fc_forwarder_test SOURCES test/gcs_fc_forwarder_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing argus::vehicle_api_testing)
argus_add_test(mavlink_gateway_fc_telemetry_relay_test SOURCES test/fc_telemetry_relay_test.cpp
               LIBS argus::mavlink_gateway_core argus::core_testing)
