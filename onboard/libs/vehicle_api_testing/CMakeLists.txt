# vehicle_api 인터페이스의 테스트용 구현: FakeVehicleTelemetry, FakeVelocitySetpointSink, FakeFlightModeControl,
# FakeFollowControl, FakeFollowControlClient, FakeFlightModeControlClient (Client Fake 는 FakePoller 로 응답 전달)
# 추종 제어 등 vehicle_api 를 쓰는 코드의 단위 테스트가 링크한다. 제품 실행 파일은 링크하지 않는다
# 헤더는 include/argus/vehicle_api_testing/ 에 두고 #include "argus/vehicle_api_testing/<파일>.hpp" 로 부른다.
# namespace 는 argus::vehicle_api_testing
add_library(argus_vehicle_api_testing INTERFACE)
add_library(argus::vehicle_api_testing ALIAS argus_vehicle_api_testing)
target_include_directories(argus_vehicle_api_testing INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_vehicle_api_testing INTERFACE argus::vehicle_api argus::core_testing)

argus_add_test(vehicle_api_testing_fakes_test SOURCES test/fakes_test.cpp LIBS argus::vehicle_api_testing)
