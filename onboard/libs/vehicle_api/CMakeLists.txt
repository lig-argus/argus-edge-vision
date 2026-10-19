# IVehicleState·IVelocitySetpoint·IModeControl·IFollowCommand 와 ZeroMQ 어댑터
# (docs/design/uml/class_follow.md, docs/spec/process_interfaces.md 4장)
# 지금은 헤더만 있는 INTERFACE 라이브러리이다. src/ 에 .cpp 가 생기면 STATIC 으로 바꾸고 소스를 넣는다.
# 헤더는 include/argus/vehicle_api/ 에 두고 #include "argus/vehicle_api/<파일>.hpp" 로 부른다. namespace 는 argus::vehicle_api
add_library(argus_vehicle_api INTERFACE)
add_library(argus::vehicle_api ALIAS argus_vehicle_api)
target_include_directories(argus_vehicle_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_vehicle_api INTERFACE argus::bus)
