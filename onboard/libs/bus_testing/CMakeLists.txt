# bus 인터페이스의 테스트용 구현 (FakeBus)
# bus 를 쓰는 코드의 단위 테스트가 링크한다. 제품 실행 파일은 링크하지 않는다
# 헤더는 include/argus/bus_testing/ 에 두고 #include "argus/bus_testing/<파일>.hpp" 로 부른다.
# namespace 는 argus::bus_testing
add_library(argus_bus_testing INTERFACE)
add_library(argus::bus_testing ALIAS argus_bus_testing)
target_include_directories(argus_bus_testing INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(argus_bus_testing INTERFACE argus::bus argus::core_testing)

argus_add_test(bus_testing_fake_bus_test SOURCES test/fake_bus_test.cpp LIBS argus::bus_testing)
