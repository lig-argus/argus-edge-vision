# IVehicleState·IVelocitySetpoint·IModeControl·IFollowCommand 순수 인터페이스
# (docs/design/uml/class_follow.md, docs/spec/process_interfaces.md 4장)
# 다른 모듈에 의존하지 않는다. 추종 제어 알고리즘과 가짜 구현(Fake*) 단위 테스트는 이 타깃만 링크해
# ZeroMQ·MessagePack 없이 빌드된다. ZeroMQ 어댑터는 libs/vehicle_api_zmq
# 헤더는 include/argus/vehicle_api/ 에 두고 #include "argus/vehicle_api/<파일>.hpp" 로 부른다. namespace 는 argus::vehicle_api
add_library(argus_vehicle_api INTERFACE)
add_library(argus::vehicle_api ALIAS argus_vehicle_api)
target_include_directories(argus_vehicle_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
