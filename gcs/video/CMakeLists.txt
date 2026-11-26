# 영상 출처 인터페이스(IVideoSource)와 값 타입. 해독한 프레임만 다루며 코덱, 녹화 형식에 묶이지 않는다.
# 구현(RTSP)은 video_rtsp 모듈에 둔다. namespace 는 argus::gcs::video
argus_gcs_add_library(video
                      SOURCES include/argus/gcs/video/video_source.hpp
                              src/video_source.cpp
                      QT Core Gui)
