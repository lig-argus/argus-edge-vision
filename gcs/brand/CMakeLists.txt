# 통제기 실행 아이콘 그리기 (LIG 워드마크와 체계 이름). namespace 는 argus::gcs::brand
argus_gcs_add_library(brand SOURCES src/app_icon.cpp DEPS theme QT Core Gui Svg)

# 테스트는 소스 폴더의 워드마크 파일을 직접 읽는다 (리소스 등록은 실행 파일 app 이 한다)
argus_gcs_add_test(gcs_brand_app_icon_test SOURCES test/app_icon_test.cpp LIBS argus::gcs::brand)
target_compile_definitions(gcs_brand_app_icon_test
                           PRIVATE ARGUS_GCS_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../resources")
