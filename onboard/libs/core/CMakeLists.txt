# IClock 등 프로세스 공통 기반 (docs/design/uml/class_onboard.md 1절)
# 지금은 헤더만 있는 INTERFACE 라이브러리이다. src/ 에 .cpp 가 생기면 STATIC 으로 바꾸고 소스를 넣는다.
# 헤더는 include/argus/core/ 에 두고 #include "argus/core/<파일>.hpp" 로 부른다. namespace 는 argus::core
add_library(argus_core INTERFACE)
add_library(argus::core ALIAS argus_core)
target_include_directories(argus_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_core INTERFACE argus_onboard_warnings)
