# vehicle_api 인터페이스의 bus 어댑터: vehicle_api 의 값 타입을 메시지(protobuf)로 변환해 bus::IBus 로
# 주고받는다 (docs/spec/process_interfaces.md 5장). 통신 구현(bus_zmq 등)에 의존하지 않는다.
# 프로세스 실행 파일이 이 타깃을 링크하고, 알고리즘 코드는 argus::vehicle_api 만 링크한다.
# 헤더는 include/argus/vehicle_api_bus/ 에 두고 #include "argus/vehicle_api_bus/<파일>.hpp" 로 부른다.
# namespace 는 argus::vehicle_api_bus
# 클래스 이름: Bus + 대상 + bus 에서의 역할. 토픽은 Publisher 와 Subscriber, 요청-응답은 Client 와 Server
add_library(argus_vehicle_api_bus STATIC
            src/bus_flight_mode_control_client.cpp
            src/bus_flight_mode_control_server.cpp
            src/bus_follow_control_client.cpp
            src/bus_follow_control_server.cpp
            src/bus_vehicle_telemetry_publisher.cpp
            src/bus_vehicle_telemetry_subscriber.cpp
            src/bus_velocity_setpoint_publisher.cpp
            src/bus_velocity_setpoint_subscriber.cpp
            src/codec_common.cpp
            src/control_mode_request_codec.cpp
            src/control_setpoint_codec.cpp
            src/fc_telemetry_codec.cpp
            src/follow_command_codec.cpp)
add_library(argus::vehicle_api_bus ALIAS argus_vehicle_api_bus)
target_include_directories(argus_vehicle_api_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# protocol 은 PRIVATE 이다. 생성된 메시지 타입은 src/ 안에서만 쓰고 헤더에 나오지 않는다
target_link_libraries(argus_vehicle_api_bus
                      PUBLIC argus::vehicle_api argus::bus argus_onboard_warnings
                      PRIVATE argus::protocol)

# 테스트는 메시지 본문을 직접 확인하므로 protocol 도 링크한다
argus_add_test(vehicle_api_bus_velocity_setpoint_test SOURCES test/velocity_setpoint_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::protocol)
argus_add_test(vehicle_api_bus_vehicle_telemetry_test SOURCES test/vehicle_telemetry_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::protocol)
argus_add_test(vehicle_api_bus_flight_mode_control_test SOURCES test/flight_mode_control_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::vehicle_api_testing
                    argus::protocol)
argus_add_test(vehicle_api_bus_follow_control_test SOURCES test/follow_control_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::vehicle_api_testing
                    argus::protocol)
