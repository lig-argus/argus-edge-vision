# RTSP 영상 수신의 해석 부품과 수신기 인터페이스 (docs/spec/video.md). GStreamer 를 쓰지 않는다.
# 수신기 구현(GStreamer)은 rtsp_gstreamer 모듈에 둔다. namespace 는 argus::gcs::rtsp
argus_gcs_add_library(rtsp
                      SOURCES include/argus/gcs/rtsp/rtsp_receiver.hpp
                              src/rtsp_receiver.cpp
                              src/onvif_metadata.cpp
                              src/metadata_reassembler.cpp
                              src/rtcp_clock.cpp
                              src/rtp_frame_times.cpp
                              src/frame_synchronizer.cpp
                              src/rtp_sequence.cpp
                      QT Core)

foreach(name onvif_metadata metadata_reassembler rtcp_clock rtp_frame_times frame_synchronizer
             rtp_sequence)
    argus_gcs_add_test(gcs_rtsp_${name}_test SOURCES test/${name}_test.cpp LIBS argus::gcs::rtsp)
endforeach()
