# 공통 기반: IClock(monotonic clock 인터페이스)과 SteadyClock. 다른 통제기 모듈에 의존하지 않는다
# 헤더는 include/argus/gcs/core/ 에 두고 #include "argus/gcs/core/<파일>.hpp" 로 부른다.
# namespace 는 argus::gcs::core
argus_gcs_add_library(core SOURCES src/steady_clock.cpp QT Core)

# SteadyClock 과 FakeClock 테스트
argus_gcs_add_test(gcs_core_clock_test SOURCES test/clock_test.cpp LIBS argus::gcs::core_testing)
