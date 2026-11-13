# bus 인터페이스의 ZeroMQ 구현: ZmqBus (bus::IBus), ZmqPoller (core::IPoller)
# (docs/design/uml/class_onboard.md 1절). ZeroMQ 로 통신하는 실행 파일만 링크한다. 인터페이스만
# 사용하는 모듈은 argus::bus 만 링크하므로 libzmq 없이 빌드된다.
# 헤더는 include/argus/bus_zmq/ 에 두고 #include "argus/bus_zmq/<파일>.hpp" 로 부른다.
# namespace 는 argus::bus_zmq
add_library(argus_bus_zmq STATIC src/zmq_bus.cpp src/zmq_poller.cpp)
add_library(argus::bus_zmq ALIAS argus_bus_zmq)
target_include_directories(argus_bus_zmq PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# cppzmq 는 PUBLIC 이다. 헤더에 zmq::context_t, zmq::socket_ref 가 나오므로 사용하는 쪽도
# cppzmq 헤더가 필요하다
target_link_libraries(argus_bus_zmq PUBLIC argus::bus cppzmq argus_onboard_warnings)

# budget 과 timeout 테스트가 FakeClock 을 사용한다
argus_add_test(bus_zmq_poller_test SOURCES test/zmq_poller_test.cpp
               LIBS argus::bus_zmq argus::core_testing)
argus_add_test(bus_zmq_bus_test SOURCES test/zmq_bus_test.cpp
               LIBS argus::bus_zmq argus::core_testing)
