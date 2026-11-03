# vehicle_api 인터페이스의 ZeroMQ 어댑터 (ZmqVehicleState·ZmqVelocitySetpointSender·ZmqModeControl 등,
# docs/design/uml/class_follow.md). 프로세스 실행 파일이 이 타깃을 링크하고, 알고리즘 코드는 링크하지 않는다
# 지금은 헤더만 있는 INTERFACE 라이브러리이다. src/ 에 .cpp 가 생기면 STATIC 으로 바꾸고 소스를 넣는다.
# 헤더는 include/argus/vehicle_api_zmq/ 에 두고 #include "argus/vehicle_api_zmq/<파일>.hpp" 로 부른다. namespace 는 argus::vehicle_api_zmq
add_library(argus_vehicle_api_zmq INTERFACE)
add_library(argus::vehicle_api_zmq ALIAS argus_vehicle_api_zmq)
target_include_directories(argus_vehicle_api_zmq INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_vehicle_api_zmq INTERFACE argus::vehicle_api argus::bus)
