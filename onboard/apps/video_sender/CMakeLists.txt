# 영상 송신부: RTSP 영상·메타데이터 트랙 (docs/design/uml/class_video.md 1절). 통신 담당
# 실행 파일 이름: argus_video_sender (main.cpp 로 add_executable). 그 밖의 소스는 라이브러리 argus_video_sender_core(STATIC)에
# 추가하고 실행 파일과 테스트가 링크한다 (CONTRIBUTING.md 0.4). 모두 argus_onboard_warnings 를 링크한다.
# 헤더는 include/argus/video_sender/, namespace 는 argus::video_sender

# 영상 송신부 안 클래스들이 쓰는 인터페이스와 값 타입 (IFrameSource, CollectedFrame, VideoEncodeConfig)
add_library(argus_video_sender_interfaces INTERFACE)
add_library(argus::video_sender_interfaces ALIAS argus_video_sender_interfaces)
target_include_directories(argus_video_sender_interfaces INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_video_sender_interfaces INTERFACE argus::core argus::perception_api
                                                              argus_onboard_warnings)

# FakeFrameSource 는 영상 송신부 테스트만 쓰므로 test/ 에 둔다
argus_add_test(video_sender_fake_frame_source_test SOURCES test/fake_frame_source_test.cpp
               LIBS argus::video_sender_interfaces argus::core_testing)

# 영상 송신부의 코드 중 GStreamer 를 사용하지 않는 부분. 실행 파일(argus_video_sender)과 이름이 겹치지
# 않도록 _core 를 붙인다. ZeroMQ, protobuf, GStreamer 를 링크하지 않으므로 테스트는 gtest 만으로 실행한다
# - FrameCollector: 같은 프레임의 프레임, 탐지 결과, 추적 결과를 CollectedFrame 하나로 출력
#   (process_interfaces.md 5.3 의 메시지 결합)
add_library(argus_video_sender_core STATIC src/frame_collector.cpp)
add_library(argus::video_sender_core ALIAS argus_video_sender_core)
target_link_libraries(argus_video_sender_core PUBLIC argus::video_sender_interfaces)

argus_add_test(video_sender_frame_collector_test SOURCES test/frame_collector_test.cpp
               LIBS argus::video_sender_core argus::core_testing)
