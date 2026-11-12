# onboard 프로세스: FC 통신, 통제기 통신 (docs/design/uml/class_onboard.md). 통신 담당
# 실행 파일 이름: argus_onboard. 첫 소스와 함께 add_executable 을 추가하고 argus_onboard_warnings 를 링크한다.
# 헤더는 include/argus/onboard/, namespace 는 argus::onboard

# onboard 안 클래스들이 쓰는 인터페이스 헤더 (IMavlinkConnection, IMavlinkCommandHandler)
add_library(argus_onboard_interfaces INTERFACE)
add_library(argus::onboard_interfaces ALIAS argus_onboard_interfaces)
target_include_directories(argus_onboard_interfaces INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_onboard_interfaces INTERFACE argus::core argus_mavlink argus_onboard_warnings)

# FakeMavlinkConnection 은 onboard 테스트만 쓰므로 test/ 에 둔다
argus_add_test(onboard_fake_mavlink_connection_test SOURCES test/fake_mavlink_connection_test.cpp
               LIBS argus::onboard_interfaces argus::core_testing)
argus_add_test(onboard_command_responder_test SOURCES test/command_responder_test.cpp
               LIBS argus::onboard_interfaces)

# main.cpp 를 제외한 onboard 의 소스. 실행 파일과 테스트가 링크한다
add_library(argus_onboard_core STATIC
    src/fc_link_tracker.cpp
    src/fc_telemetry_receiver.cpp
    src/px4_flight_mode.cpp
    src/time_sync.cpp
    src/udp_mavlink_connection.cpp)
add_library(argus::onboard_core ALIAS argus_onboard_core)
target_link_libraries(argus_onboard_core
    PUBLIC argus::onboard_interfaces argus::vehicle_api argus_onboard_warnings)

argus_add_test(onboard_handler_list_test SOURCES test/handler_list_test.cpp
               LIBS argus::onboard_interfaces)
argus_add_test(onboard_health_state_test SOURCES test/health_state_test.cpp
               LIBS argus::onboard_interfaces)
argus_add_test(onboard_px4_flight_mode_test SOURCES test/px4_flight_mode_test.cpp
               LIBS argus::onboard_core)
argus_add_test(onboard_mavlink_handler_registry_test SOURCES test/mavlink_handler_registry_test.cpp
               LIBS argus::onboard_interfaces)
# UdpMavlinkConnection 테스트는 loopback UDP 소켓을 실제로 연다
argus_add_test(onboard_udp_mavlink_connection_test SOURCES test/udp_mavlink_connection_test.cpp
               LIBS argus::onboard_core)
argus_add_test(onboard_time_sync_test SOURCES test/time_sync_test.cpp
               LIBS argus::onboard_core argus::core_testing)
argus_add_test(onboard_fc_link_tracker_test SOURCES test/fc_link_tracker_test.cpp
               LIBS argus::onboard_core argus::core_testing)
argus_add_test(onboard_fc_telemetry_receiver_test SOURCES test/fc_telemetry_receiver_test.cpp
               LIBS argus::onboard_core argus::core_testing)
