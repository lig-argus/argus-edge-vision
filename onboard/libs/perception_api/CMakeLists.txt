# 탐지·추적 결과 값 타입 (docs/spec/process_interfaces.md 5.1, docs/design/uml/class_video.md 1절)
# 영상 송신부, mavlink_gateway(LatestTracks), 추적기가 함께 쓴다. 조회와 알림 인터페이스
# (tracking_status_source.hpp, tracking_status_events.hpp)를 함께 두며, 알림의 등록 해제에
# core::Subscription 을 쓰므로 core 에만 의존한다.
# 메시지(protobuf, IDL) ↔ 구조체 변환은 perception_api_bus 가 한다
# 헤더는 include/argus/perception_api/ 에 두고 #include "argus/perception_api/<파일>.hpp" 로 부른다.
# namespace 는 argus::perception_api
add_library(argus_perception_api INTERFACE)
add_library(argus::perception_api ALIAS argus_perception_api)
target_include_directories(argus_perception_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_perception_api INTERFACE argus::core argus_onboard_warnings)

argus_add_test(perception_api_types_test SOURCES test/types_test.cpp LIBS argus::perception_api)
