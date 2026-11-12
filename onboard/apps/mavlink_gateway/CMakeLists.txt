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
    src/fc_link_tracker.cpp
    src/fc_telemetry_receiver.cpp
    src/px4_flight_mode.cpp
    src/time_sync.cpp
    src/udp_mavlink_connection.cpp)
add_library(argus::mavlink_gateway_core ALIAS argus_mavlink_gateway_core)
target_link_libraries(argus_mavlink_gateway_core
    PUBLIC argus::mavlink_gateway_interfaces argus::vehicle_api argus_onboard_warnings)

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
