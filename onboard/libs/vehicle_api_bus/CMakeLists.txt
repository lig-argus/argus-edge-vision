# vehicle_api 인터페이스의 bus 어댑터: vehicle_api 의 값 타입을 bus::IBus 로 주고받는다
# (docs/spec/process_interfaces.md 5장). 타깃은 둘이다.
# - argus::vehicle_api_bus: 어댑터와 메시지 타입(message_types.hpp). 통신 구현(bus_zmq 등)과
#   protobuf 를 include 하지 않는다. 프로세스 실행 파일이 링크하고, 알고리즘 코드는
#   argus::vehicle_api 만 링크한다.
# - argus::vehicle_api_bus_zmq: 값 타입을 메시지(protobuf)로 바꾸는 ZmqBus 용 변환 표
#   (zmq_codecs.hpp). ZeroMQ 로 통신하는 실행 파일만 링크한다. 통신 구현을 추가하면 그 구현의 변환
#   타깃을 따로 둔다.
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
            src/bus_velocity_setpoint_subscriber.cpp)
add_library(argus::vehicle_api_bus ALIAS argus_vehicle_api_bus)
target_include_directories(argus_vehicle_api_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 토픽 이름과 큐 길이 상수(argus/protocol/topics.hpp)만 쓰므로 protocol_names 를 링크한다.
# argus::protocol 을 링크하면 protobuf 를 쓰지 않는 통신 구현의 빌드에도 protobuf 가 필요해진다
target_link_libraries(argus_vehicle_api_bus
                      PUBLIC argus::vehicle_api argus::bus argus_onboard_warnings
                      PRIVATE argus::protocol_names)

add_library(argus_vehicle_api_bus_zmq STATIC
            src/codec_common.cpp
            src/control_mode_request_codec.cpp
            src/control_setpoint_codec.cpp
            src/fc_telemetry_codec.cpp
            src/follow_command_codec.cpp
            src/zmq_codecs.cpp)
add_library(argus::vehicle_api_bus_zmq ALIAS argus_vehicle_api_bus_zmq)
# protocol 은 PRIVATE 이다. 생성된 메시지 타입은 src/ 안에서만 쓰고 헤더에 나오지 않는다.
# protocol 을 bus_zmq 보다 먼저 적는다. include 경로가 적은 순서대로 붙는데, cppzmq 의 include
# 경로(macOS Homebrew 는 /opt/homebrew/include)에 다른 버전의 protobuf 헤더가 있으면 그 헤더가 먼저
# 잡혀 생성 코드와 맞지 않는다. protobuf 와 bus_zmq 를 함께 링크하는 테스트도 같은 순서로 적는다
target_link_libraries(argus_vehicle_api_bus_zmq
                      PRIVATE argus::protocol
                      PUBLIC argus::vehicle_api_bus argus::bus_zmq argus_onboard_warnings)

# 어댑터 테스트는 FakeBus 로 값 타입을 주고받는다. protobuf 를 링크하지 않는다
argus_add_test(vehicle_api_bus_velocity_setpoint_test SOURCES test/velocity_setpoint_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::protocol_names)
argus_add_test(vehicle_api_bus_vehicle_telemetry_test SOURCES test/vehicle_telemetry_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::protocol_names)
argus_add_test(vehicle_api_bus_flight_mode_control_test SOURCES test/flight_mode_control_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::vehicle_api_testing)
argus_add_test(vehicle_api_bus_follow_control_test SOURCES test/follow_control_test.cpp
               LIBS argus::vehicle_api_bus argus::bus_testing argus::vehicle_api_testing)
# 변환 표 테스트는 메시지 본문을 직접 확인하므로 protocol 도 링크한다
argus_add_test(vehicle_api_bus_zmq_codecs_test SOURCES test/zmq_codecs_test.cpp
               LIBS argus::protocol argus::vehicle_api_bus_zmq)
# 어댑터를 ZmqBus 에 붙여 전송 바이트까지 확인한다
argus_add_test(vehicle_api_bus_zmq_end_to_end_test SOURCES test/zmq_end_to_end_test.cpp
               LIBS argus::protocol argus::vehicle_api_bus_zmq argus::core_testing
                    argus::vehicle_api_testing)
