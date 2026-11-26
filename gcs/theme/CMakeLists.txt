# 통제기 화면의 디자인 토큰(색, 크기)과 움직임 값. 헤더만 있다.
# namespace 는 argus::gcs::theme, argus::gcs::motion
argus_gcs_add_library(theme HEADER_ONLY QT Core Gui)

argus_gcs_add_test(gcs_theme_test SOURCES test/theme_test.cpp LIBS argus::gcs::theme)
