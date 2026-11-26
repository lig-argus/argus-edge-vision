# IRtspReceiver 의 GStreamer 구현 (GstRtspReceiver). rtspsrc 와 해독 요소(plugins-good, gst-libav)가 필요하다.
# src/ 의 track_branch, decode_elements 는 이 모듈 안에서만 쓰며 테스트만 src/ 를 include 경로에 더한다.
# namespace 는 argus::gcs::rtsp_gstreamer
argus_gcs_add_library(rtsp_gstreamer
                      SOURCES include/argus/gcs/rtsp_gstreamer/gst_rtsp_receiver.hpp
                              src/gst_rtsp_receiver.cpp
                              src/track_branch.cpp
                              src/decode_elements.cpp
                      DEPS rtsp core
                      QT Core
                      EXTERNAL gstreamer)

foreach(name track_branch gst_rtsp_receiver)
    argus_gcs_add_test(gcs_rtsp_gstreamer_${name}_test SOURCES test/${name}_test.cpp
                       LIBS argus::gcs::rtsp_gstreamer argus::gcs::core_testing)
    target_include_directories(gcs_rtsp_gstreamer_${name}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endforeach()
