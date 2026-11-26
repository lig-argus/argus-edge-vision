# 통제기 모듈별 허용 의존 (한 곳에 정의한다)
#   argus_gcs_module_rule(<모듈> [DEPS <모듈>...] [QT <Qt 구성 요소>...] [EXTERNAL mavlink|gstreamer...])
#   DEPS: 이 모듈이 링크하고 include 할 수 있는 통제기 모듈
#   QT: 이 모듈이 링크할 수 있는 Qt 구성 요소. DEPS 의 모듈이 쓰는 구성 요소도 여기에 있어야 한다
#       (의존 모듈의 헤더가 그 구성 요소를 함께 가져오기 때문)
#   EXTERNAL: 이 모듈이 링크할 수 있는 외부 라이브러리
#
# 규칙
# - 명령 규칙, 기체 상태 판단(link), 화면 표시 규칙(view)은 소켓(Network), GStreamer, QML 을 쓰지 않는다.
#   기체, 네트워크, 화면 없이 테스트하기 위해서다
# - 인터페이스와 구현은 다른 모듈에 둔다: mavlink_connection / net, rtsp / rtsp_gstreamer,
#   video / video_rtsp, recording / recording_file. 구현을 만들어 넘기는 곳은 app 하나다
# - <모듈>_testing 은 테스트만 링크한다
# - qml 폴더(.qml 파일)는 quick 모듈의 QML 모듈로 묶는다

argus_gcs_module_rule(core QT Core)
argus_gcs_module_rule(core_testing DEPS core QT Core)

argus_gcs_module_rule(mavlink_connection DEPS core QT Core EXTERNAL mavlink)
argus_gcs_module_rule(mavlink_connection_testing DEPS mavlink_connection core QT Core EXTERNAL mavlink)
argus_gcs_module_rule(net DEPS mavlink_connection core QT Core Network EXTERNAL mavlink)

argus_gcs_module_rule(link DEPS mavlink_connection core QT Core EXTERNAL mavlink)

argus_gcs_module_rule(rtsp DEPS core QT Core)
argus_gcs_module_rule(video DEPS core QT Core Gui)
argus_gcs_module_rule(video_rtsp DEPS video rtsp link core QT Core Gui)
argus_gcs_module_rule(recording DEPS link video core QT Core Gui)
argus_gcs_module_rule(recording_file DEPS recording core QT Core Gui)
argus_gcs_module_rule(rtsp_gstreamer DEPS rtsp video recording core QT Core Gui EXTERNAL gstreamer)

argus_gcs_module_rule(theme QT Core Gui)
argus_gcs_module_rule(view DEPS link video core QT Core Gui)
argus_gcs_module_rule(render DEPS view theme core QT Core Gui)
argus_gcs_module_rule(brand DEPS theme QT Core Gui Svg)
argus_gcs_module_rule(quick DEPS view render recording video link theme brand core
                      QT Core Gui Qml Quick Quick3D Svg)

argus_gcs_module_rule(app
                      DEPS core mavlink_connection net link rtsp rtsp_gstreamer video video_rtsp recording
                           recording_file theme view render brand quick
                      QT Core Gui Network Qml Quick Quick3D Svg
                      EXTERNAL mavlink gstreamer)
