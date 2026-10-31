# 탐지·추적 결과 값 타입 (docs/spec/process_interfaces.md 5.1, docs/design/uml/class_video.md 1절)
# 영상 송신부, onboard(LatestTracks), 추적기가 함께 쓴다. 다른 모듈에 의존하지 않는다.
# 메시지(MessagePack) ↔ 구조체 변환은 어댑터가 한다 (다음 PR)
# 헤더는 include/argus/perception_api/ 에 두고 #include "argus/perception_api/<파일>.hpp" 로 부른다.
# namespace 는 argus::perception_api
add_library(argus_perception_api INTERFACE)
add_library(argus::perception_api ALIAS argus_perception_api)
target_include_directories(argus_perception_api INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_perception_api INTERFACE argus_onboard_warnings)

argus_add_test(perception_api_types_test SOURCES test/types_test.cpp LIBS argus::perception_api)
