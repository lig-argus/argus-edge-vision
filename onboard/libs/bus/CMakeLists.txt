# ZeroMQ PUB/SUB·REQ/REP 인터페이스(IPublisher·ISubscriber·IRequester·IReplier)와 구현,
# MessagePack 메시지 타입 (docs/design/uml/class_onboard.md 1절)
# 지금은 헤더만 있는 INTERFACE 라이브러리이다. src/ 에 .cpp 가 생기면 STATIC 으로 바꾸고 소스를 넣는다.
# 헤더는 include/argus/bus/ 에 두고 #include "argus/bus/<파일>.hpp" 로 부른다. namespace 는 argus::bus
add_library(argus_bus INTERFACE)
add_library(argus::bus ALIAS argus_bus)
target_include_directories(argus_bus INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_bus INTERFACE argus::core cppzmq msgpack-cxx)
