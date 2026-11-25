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
    src/fc_command_policy.cpp
    src/fc_control.cpp
    src/fc_gateway.cpp
    src/fc_link_tracker.cpp
    src/fc_telemetry_receiver.cpp
    src/mode_command_encoding.cpp
    src/px4_flight_mode.cpp
    src/setpoint_encoding.cpp
    src/time_sync.cpp
    src/udp_mavlink_connection.cpp)
add_library(argus::mavlink_gateway_core ALIAS argus_mavlink_gateway_core)
# FcGateway 가 bus 인터페이스와 vehicle_api 의 bus 어댑터를 사용한다. 통신 구현(bus_zmq)은 링크하지
# 않는다 (실행 파일만 링크)
target_link_libraries(argus_mavlink_gateway_core
    PUBLIC argus::mavlink_gateway_interfaces argus::vehicle_api argus::bus argus::vehicle_api_bus
           argus_onboard_warnings)

# 통신 구현(ZeroMQ, UDP)과 실제 시계는 main.cpp 에서만 생성한다
# (docs/design/uml/class_mavlink_gateway.md 4절)
add_executable(argus_mavlink_gateway src/main.cpp)
target_link_libraries(argus_mavlink_gateway
    PRIVATE argus::mavlink_gateway_core argus::vehicle_api_bus_zmq argus::protocol_names
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
