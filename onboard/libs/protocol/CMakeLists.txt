# RPi 안 ZeroMQ 토픽 이름, RPC 이름, 기본 엔드포인트 상수 (docs/spec/process_interfaces.md 5장)
# 다른 모듈에 의존하지 않는다. 헤더만 있는 INTERFACE 라이브러리이다.
# 헤더는 include/argus/protocol/ 에 두고 #include "argus/protocol/<파일>.hpp" 로 부른다. namespace 는 argus::protocol
add_library(argus_protocol INTERFACE)
add_library(argus::protocol ALIAS argus_protocol)
target_include_directories(argus_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_protocol INTERFACE argus_onboard_warnings)

argus_add_test(protocol_test SOURCES test/protocol_test.cpp LIBS argus::protocol)
