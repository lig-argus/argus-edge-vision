# 영상 스트림 정보의 값 타입과 조회 인터페이스 (docs/spec/process_interfaces.md 5.1.6,
# docs/design/uml/class_video.md, docs/design/uml/class_mavlink_gateway.md 3절)
# 발행하는 영상 송신부와 구독하는 mavlink_gateway 가 함께 쓴다. 두 실행 파일 어느 쪽에도 두지 않는 이유:
# 한쪽에 두면 다른 쪽이 그 실행 파일의 코드에 의존하게 된다. bus 어댑터는 libs/video_api_bus
# 센서는 vehicle_api::Sensor 를 쓴다 (값이 MAVLink ARGUS_SENSOR 와 같다)
# 헤더는 include/argus/video_api/ 에 두고 #include "argus/video_api/<파일>.hpp" 로 부른다.
# namespace 는 argus::video_api
add_library(argus_video_api INTERFACE)
add_library(argus::video_api ALIAS argus_video_api)
target_include_directories(argus_video_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_video_api INTERFACE argus::vehicle_api argus_onboard_warnings)

argus_add_test(video_api_types_test SOURCES test/types_test.cpp LIBS argus::video_api)
