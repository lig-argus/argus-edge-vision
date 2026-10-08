# MAVLink C 헤더 (tools/generate_mavlink.sh 로 생성, 헤더만 존재)
# onboard, 통제기와 그 테스트 코드가 쓴다.
# SYSTEM: 생성 코드의 경고와 clang-tidy 진단을 우리 설정에서 제외한다
add_library(argus_mavlink INTERFACE)
target_include_directories(argus_mavlink SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/generated)
