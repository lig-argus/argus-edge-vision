# 영상 송신부: RTSP 영상·메타데이터 트랙 (docs/design/uml/class_video.md 1절). 통신 담당
# 실행 파일 이름: argus_video_sender. 첫 소스와 함께 add_executable 을 추가하고 argus_onboard_warnings 를 링크한다.
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
