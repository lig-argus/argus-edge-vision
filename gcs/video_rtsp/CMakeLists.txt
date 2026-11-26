# IVideoSource 의 RTSP 구현 (RtspVideoSource). 수신기(IRtspReceiver)는 생성 함수로 받는다.
# namespace 는 argus::gcs::video_rtsp
argus_gcs_add_library(video_rtsp
                      SOURCES include/argus/gcs/video_rtsp/rtsp_video_source.hpp
                              src/rtsp_video_source.cpp
                      DEPS video rtsp core
                      QT Core Gui)

argus_gcs_add_test(gcs_video_rtsp_rtsp_video_source_test
                   SOURCES test/rtsp_video_source_test.cpp test/fake_rtsp_receiver.hpp
                   LIBS argus::gcs::video_rtsp argus::gcs::core_testing)
