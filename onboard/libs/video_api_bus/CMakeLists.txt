# video_api 의 bus 어댑터: video_api 의 값 타입을 bus::IBus 로 주고받는다
# (docs/spec/process_interfaces.md 5.1.6). 타깃은 셋이다.
# - argus::video_api_bus: 발행 클래스, 구독 클래스, 메시지 타입(message_types.hpp). 통신 구현과
#   protobuf 를 include 하지 않는다. 프로세스 실행 파일과 그 코드의 테스트가 링크한다.
# - argus::video_api_bus_zmq: 값 타입을 메시지(protobuf)로 바꾸는 ZmqBus 용 변환 표
#   (zmq_codecs.hpp). ZeroMQ 로 통신하는 실행 파일만 링크한다.
# - argus::video_api_bus_dds: 값 타입을 IDL 타입의 표본으로 바꾸는 DdsBus 용 변환 표
#   (dds_types.hpp). Fast DDS 로 통신하는 실행 파일만 링크한다. onboard/CMakeLists.txt 의
#   ARGUS_BUILD_BUS_DDS 가 ON 일 때만 빌드한다.
# 헤더는 include/argus/video_api_bus/ 에 두고 #include "argus/video_api_bus/<파일>.hpp" 로 부른다.
# namespace 는 argus::video_api_bus
# 클래스 이름: Bus + 대상 + bus 에서의 역할 (토픽은 Publisher 와 Subscriber)
add_library(argus_video_api_bus STATIC
            src/bus_video_stream_info_publisher.cpp
            src/bus_video_stream_info_subscriber.cpp)
add_library(argus::video_api_bus ALIAS argus_video_api_bus)
target_include_directories(argus_video_api_bus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 토픽 이름과 큐 길이 상수(argus/protocol/topics.hpp)만 쓰므로 protocol_names 를 링크한다.
# argus::protocol 을 링크하면 protobuf 를 쓰지 않는 통신 구현의 빌드에도 protobuf 가 필요해진다
target_link_libraries(argus_video_api_bus
                      PUBLIC argus::video_api argus::bus argus::core argus_onboard_warnings
                      PRIVATE argus::protocol_names)

add_library(argus_video_api_bus_zmq STATIC
            src/video_stream_info_codec.cpp
            src/zmq_codecs.cpp)
add_library(argus::video_api_bus_zmq ALIAS argus_video_api_bus_zmq)
# protocol 은 PRIVATE 이다. 생성된 메시지 타입은 src/ 안에서만 쓰고 헤더에 나오지 않는다.
# protocol 을 bus_zmq 보다 먼저 적는다. include 경로가 적은 순서대로 붙는데, cppzmq 의 include
# 경로(macOS Homebrew 는 /opt/homebrew/include)에 다른 버전의 protobuf 헤더가 있으면 그 헤더가 먼저
# 잡혀 생성 코드와 맞지 않는다. protobuf 와 bus_zmq 를 함께 링크하는 테스트도 같은 순서로 적는다
target_link_libraries(argus_video_api_bus_zmq
                      PRIVATE argus::protocol
                      PUBLIC argus::video_api_bus argus::bus_zmq argus_onboard_warnings)

# 발행 클래스와 구독 클래스의 테스트는 FakeBus 로 값 타입을 주고받는다. protobuf 를 링크하지 않는다
argus_add_test(video_api_bus_video_stream_info_test SOURCES test/video_stream_info_test.cpp
               LIBS argus::video_api_bus argus::bus_testing argus::core_testing
                    argus::protocol_names)
# 변환 표 테스트는 메시지 본문을 직접 확인하므로 protocol 도 링크한다
argus_add_test(video_api_bus_zmq_codecs_test SOURCES test/zmq_codecs_test.cpp
               LIBS argus::protocol argus::video_api_bus_zmq)

if(ARGUS_BUILD_BUS_DDS)
    add_library(argus_video_api_bus_dds STATIC
                src/dds_types.cpp
                src/video_stream_info_idl.cpp)
    add_library(argus::video_api_bus_dds ALIAS argus_video_api_bus_dds)
    # protocol_idl 은 PRIVATE 이다. 생성된 IDL 타입은 src/ 안에서만 사용하고 헤더에 나오지 않는다
    # protocol_names 는 dds_topic_qos() 의 토픽 이름에 사용한다
    target_link_libraries(argus_video_api_bus_dds
                          PRIVATE argus::protocol_idl argus::protocol_names
                          PUBLIC argus::video_api_bus argus::bus_dds argus_onboard_warnings)

    # IDL 경로와 .proto 경로로 각각 바꾼 결과를 비교하므로 protocol 과 video_api_bus_zmq 도 링크한다
    argus_add_test(video_api_bus_dds_types_test SOURCES test/dds_types_test.cpp
                   LIBS argus::protocol argus::protocol_idl argus::video_api_bus_dds
                        argus::video_api_bus_zmq)
endif()
