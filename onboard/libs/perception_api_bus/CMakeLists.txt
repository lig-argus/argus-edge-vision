# perception_api 의 bus 어댑터: vision/frame/gray8, perception/observations, tracking/tracks,
# tracking/status, vision/status 를 perception_api 의 값 타입으로 받는다
# (docs/spec/process_interfaces.md 5.1). 타깃은 셋이다.
# - argus::perception_api_bus: 구독자 생성 함수, BusTrackingStatusSubscriber, BusVisionStatusSubscriber 와
#   메시지 타입(message_types.hpp). 통신 구현(bus_zmq 등)과
#   protobuf 를 include 하지 않는다. 프로세스 실행 파일과 그 조립 클래스가 링크한다.
# - argus::perception_api_bus_zmq: 값 타입을 메시지(protobuf)로 바꾸는 ZmqBus 용 변환 표
#   (zmq_codecs.hpp). ZeroMQ 로 통신하는 실행 파일만 링크한다.
# - argus::perception_api_bus_dds: 값 타입을 IDL 타입의 표본으로 바꾸는 DdsBus 용 변환 표
#   (dds_types.hpp). Fast DDS 로 통신하는 실행 파일만 링크한다. onboard/CMakeLists.txt 의
#   ARGUS_BUILD_BUS_DDS 가 ON 일 때만 빌드한다.
# 헤더는 include/argus/perception_api_bus/ 에 두고 #include "argus/perception_api_bus/<파일>.hpp" 로
# 부른다. namespace 는 argus::perception_api_bus
add_library(argus_perception_api_bus STATIC src/bus_tracking_status_subscriber.cpp
                                            src/bus_vision_status_subscriber.cpp src/subscribers.cpp)
add_library(argus::perception_api_bus ALIAS argus_perception_api_bus)
target_include_directories(argus_perception_api_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 토픽 이름과 큐 길이 상수(argus/protocol/topics.hpp)만 쓰므로 protocol_names 를 링크한다
target_link_libraries(argus_perception_api_bus
                      PUBLIC argus::perception_api argus::bus argus_onboard_warnings
                      PRIVATE argus::protocol_names)

add_library(argus_perception_api_bus_zmq STATIC src/zmq_codecs.cpp)
add_library(argus::perception_api_bus_zmq ALIAS argus_perception_api_bus_zmq)
# protocol 을 bus_zmq 보다 먼저 적는다 (vehicle_api_bus 의 CMakeLists.txt 와 같은 이유: 다른 버전의
# protobuf 헤더가 먼저 잡히지 않게)
target_link_libraries(argus_perception_api_bus_zmq
                      PRIVATE argus::protocol
                      PUBLIC argus::perception_api_bus argus::bus_zmq argus_onboard_warnings)

# 구독자 테스트는 FakeBus 로 값 타입을 주고받는다. protobuf 를 링크하지 않는다
argus_add_test(perception_api_bus_subscribers_test SOURCES test/subscribers_test.cpp
               LIBS argus::perception_api_bus argus::bus_testing argus::protocol_names)
argus_add_test(perception_api_bus_vision_status_subscriber_test
               SOURCES test/vision_status_subscriber_test.cpp
               LIBS argus::perception_api_bus argus::bus_testing argus::core_testing
                    argus::protocol_names)
argus_add_test(perception_api_bus_tracking_status_subscriber_test
               SOURCES test/tracking_status_subscriber_test.cpp
               LIBS argus::perception_api_bus argus::bus_testing argus::core_testing
                    argus::protocol_names)
# 변환 표 테스트는 메시지 본문을 직접 확인하므로 protocol 도 링크한다
argus_add_test(perception_api_bus_zmq_codecs_test SOURCES test/zmq_codecs_test.cpp
               LIBS argus::protocol argus::perception_api_bus_zmq)
# 구독자를 ZmqBus 에 붙여 multipart 로 오가는지 확인한다
argus_add_test(perception_api_bus_zmq_end_to_end_test SOURCES test/zmq_end_to_end_test.cpp
               LIBS argus::perception_api_bus_zmq argus::core_testing argus::protocol_names)

if(ARGUS_BUILD_BUS_DDS)
    add_library(argus_perception_api_bus_dds STATIC src/dds_types.cpp)
    add_library(argus::perception_api_bus_dds ALIAS argus_perception_api_bus_dds)
    # protocol_idl 은 PRIVATE 이다. 생성된 IDL 타입은 src/ 안에서만 사용하고 헤더에 나오지 않는다
    target_link_libraries(argus_perception_api_bus_dds
                          PRIVATE argus::protocol_idl
                          PUBLIC argus::perception_api_bus argus::bus_dds argus_onboard_warnings)

    # IDL 경로와 .proto 경로로 각각 바꾼 결과를 비교하므로 protocol 과 perception_api_bus_zmq 도 링크한다
    argus_add_test(perception_api_bus_dds_types_test SOURCES test/dds_types_test.cpp
                   LIBS argus::protocol argus::protocol_idl argus::perception_api_bus_dds
                        argus::perception_api_bus_zmq)
    # 구독자를 DdsBus 에 붙여 값이 오가는지 확인한다
    argus_add_test(perception_api_bus_dds_end_to_end_test SOURCES test/dds_end_to_end_test.cpp
                   LIBS argus::perception_api_bus_dds argus::protocol_names)
endif()
