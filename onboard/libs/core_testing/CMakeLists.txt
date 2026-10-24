# core 인터페이스의 테스트용 구현 (FakeClock)
# 시간을 다루는 코드의 단위 테스트가 링크한다. 제품 실행 파일은 링크하지 않는다
# 헤더는 include/argus/core_testing/ 에 두고 #include "argus/core_testing/<파일>.hpp" 로 부른다.
# namespace 는 argus::core_testing
add_library(argus_core_testing INTERFACE)
add_library(argus::core_testing ALIAS argus_core_testing)
target_include_directories(argus_core_testing INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_core_testing INTERFACE argus::core)

argus_add_test(core_testing_fake_clock_test SOURCES test/fake_clock_test.cpp LIBS argus::core_testing)
