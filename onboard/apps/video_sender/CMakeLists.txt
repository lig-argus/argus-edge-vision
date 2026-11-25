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

# 영상 송신부의 코드 중 GStreamer 를 사용하지 않는 부분. 실행 파일(argus_video_sender)과 이름이 겹치지
# 않도록 _core 를 붙인다. ZeroMQ, protobuf, GStreamer 를 링크하지 않으므로 테스트는 gtest 만으로 실행한다
# - FrameCollector: 같은 프레임의 프레임, 탐지 결과, 추적 결과를 CollectedFrame 하나로 출력
#   (process_interfaces.md 5.3 의 메시지 결합)
# - OnvifMetadataSerializer, MetadataRtpPacketizer: CollectedFrame → ONVIF XML → RTP 패킷
#   (docs/spec/video.md 3장)
# - VideoSender: FrameCollector 의 출력을 IStreamSink 로 송신
# - TestPatternFrameSource, test_pattern_scene: 카메라와 탐지 프로세스 없이 송신을 확인하기 위한
#   시험용 입력
# - stream_timing.hpp: RTP 타임스탬프와 PTS 계산 (헤더만)
# - BusFrameSource, VideoBusInput: bus 의 프레임, 탐지 결과, 추적 결과를 FrameCollector 에 넣음.
#   bus 인터페이스만 쓰고 통신 구현(bus_zmq)은 링크하지 않는다 (실행 파일만 링크)
add_library(argus_video_sender_core STATIC
            src/frame_collector.cpp src/onvif_metadata_serializer.cpp src/metadata_rtp_packetizer.cpp
            src/video_sender.cpp src/test_pattern_scene.cpp src/test_pattern_frame_source.cpp
            src/bus_frame_source.cpp src/video_bus_input.cpp)
add_library(argus::video_sender_core ALIAS argus_video_sender_core)
target_link_libraries(argus_video_sender_core
                      PUBLIC argus::video_sender_interfaces argus::perception_api_bus)

# RtspServer: IStreamSink 의 gst-rtsp-server 구현. GStreamer 를 링크하는 유일한 라이브러리이다
# (PkgConfig::ARGUS_GSTREAMER 는 onboard/CMakeLists.txt 에서 탐색한다). GStreamer 를 사용하지 않는
# 코드의 테스트가 GStreamer 를 링크하지 않도록 argus_video_sender_core 와 분리한다
add_library(argus_video_sender_rtsp STATIC src/rtsp_server.cpp)
add_library(argus::video_sender_rtsp ALIAS argus_video_sender_rtsp)
target_link_libraries(argus_video_sender_rtsp PUBLIC argus::video_sender_core
                                              PRIVATE PkgConfig::ARGUS_GSTREAMER)

# 통신 구현(ZeroMQ)과 실제 시계는 main.cpp 에서만 생성한다
add_executable(argus_video_sender src/main.cpp)
target_link_libraries(argus_video_sender
                      PRIVATE argus::video_sender_rtsp argus::perception_api_bus_zmq
                              argus::protocol_names argus_onboard_warnings)

# FakeFrameSource, FakeStreamSink 는 영상 송신부 테스트만 사용하므로 test/ 에 둔다
argus_add_test(video_sender_fake_frame_source_test SOURCES test/fake_frame_source_test.cpp
               LIBS argus::video_sender_interfaces argus::core_testing)
argus_add_test(video_sender_video_encode_config_test SOURCES test/video_encode_config_test.cpp
               LIBS argus::video_sender_interfaces)
argus_add_test(video_sender_frame_collector_test SOURCES test/frame_collector_test.cpp
               LIBS argus::video_sender_core argus::core_testing)
argus_add_test(video_sender_onvif_metadata_serializer_test
               SOURCES test/onvif_metadata_serializer_test.cpp LIBS argus::video_sender_core)
argus_add_test(video_sender_metadata_rtp_packetizer_test
               SOURCES test/metadata_rtp_packetizer_test.cpp LIBS argus::video_sender_core)
argus_add_test(video_sender_video_sender_test SOURCES test/video_sender_test.cpp
               LIBS argus::video_sender_core argus::core_testing)
argus_add_test(video_sender_test_pattern_frame_source_test
               SOURCES test/test_pattern_frame_source_test.cpp
               LIBS argus::video_sender_core argus::core_testing)
argus_add_test(video_sender_test_pattern_scene_test SOURCES test/test_pattern_scene_test.cpp
               LIBS argus::video_sender_core)
argus_add_test(video_sender_stream_timing_test SOURCES test/stream_timing_test.cpp
               LIBS argus::video_sender_core)
argus_add_test(video_sender_bus_frame_source_test SOURCES test/bus_frame_source_test.cpp
               LIBS argus::video_sender_core argus::bus_testing argus::core_testing
                    argus::protocol_names)
argus_add_test(video_sender_video_bus_input_test SOURCES test/video_bus_input_test.cpp
               LIBS argus::video_sender_core argus::bus_testing argus::core_testing
                    argus::protocol_names)
# 테스트가 RTSP 클라이언트(rtspsrc)를 직접 구성하므로 GStreamer 를 함께 링크한다
argus_add_test(video_sender_rtsp_server_test SOURCES test/rtsp_server_test.cpp
               LIBS argus::video_sender_rtsp PkgConfig::ARGUS_GSTREAMER)
