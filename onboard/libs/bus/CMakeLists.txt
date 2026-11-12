# 통신 인터페이스: IBus 가 만드는 ITopicPublisher, ITopicSubscriber, IRequester, IReplier
# (docs/design/uml/class_onboard.md 1절). 통신 구현(ZeroMQ, Fast DDS 등)을 교체해도 이 인터페이스는
# 바뀌지 않는다. bus 는 타입 있는 값을 받고, 전송 형식으로 바꾸는 일은 통신 구현이 한다.
# 헤더만 있는 INTERFACE 라이브러리이다. ZeroMQ 구현(ZmqBus)은 libs/bus_zmq 에 있고, 인터페이스만
# 사용하는 코드는 ZeroMQ 없이 빌드된다.
# 헤더는 include/argus/bus/ 에 두고 #include "argus/bus/<파일>.hpp" 로 부른다. namespace 는 argus::bus
add_library(argus_bus INTERFACE)
add_library(argus::bus ALIAS argus_bus)
target_include_directories(argus_bus INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_bus INTERFACE argus::core argus_onboard_warnings)

# typed_test 는 래퍼를 테스트용 구현(FakeBus)에 붙여 확인한다
argus_add_test(bus_message_type_test SOURCES test/message_type_test.cpp LIBS argus::bus)
argus_add_test(bus_typed_test SOURCES test/typed_test.cpp LIBS argus::bus argus::bus_testing)
