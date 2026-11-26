# core 인터페이스의 테스트용 구현 (FakeClock). 시각을 다루는 모듈의 테스트가 링크한다.
# 제품 실행 파일은 링크하지 않는다. namespace 는 argus::gcs::core_testing
argus_gcs_add_library(core_testing HEADER_ONLY DEPS core QT Core)
