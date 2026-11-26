# 통제기 모듈의 include 가 선언한 의존(DEPS)을 넘지 않는지 확인한다 (테스트 gcs_module_deps_test)
#   cmake -DMODULE_LIST=<빌드 폴더>/argus_gcs_modules.cmake -P check_module_deps.cmake
# 규칙
# - include/, src/ 의 파일은 자기 모듈과 DEPS 의 모듈만 include 한다 (#include "argus/gcs/<모듈>/...")
# - test/ 의 파일은 위에 더해 그 모듈들의 <모듈>_testing 을 include 할 수 있다
# 링크만으로는 의존 모듈이 PUBLIC 으로 가져온 다른 모듈의 헤더도 보이므로, 선언하지 않은 의존을 여기서 잡는다
include(${MODULE_LIST})

set(violations "")
foreach(module IN LISTS ARGUS_GCS_MODULES)
    set(dir ${ARGUS_GCS_MODULE_${module}_DIR})
    set(allowed ${module} ${ARGUS_GCS_MODULE_${module}_DEPS})
    set(allowed_in_test ${allowed})
    foreach(name IN LISTS allowed)
        list(APPEND allowed_in_test ${name}_testing)
    endforeach()

    file(GLOB_RECURSE files ${dir}/include/*.hpp ${dir}/src/*.hpp ${dir}/src/*.cpp ${dir}/test/*.hpp
         ${dir}/test/*.cpp)
    foreach(file IN LISTS files)
        if(file MATCHES "^${dir}/test/")
            set(permitted ${allowed_in_test})
        else()
            set(permitted ${allowed})
        endif()
        file(STRINGS ${file} lines REGEX "^[ \t]*#[ \t]*include[ \t]*\"argus/gcs/")
        foreach(line IN LISTS lines)
            string(REGEX MATCH "argus/gcs/([a-z_]+)/" match "${line}")
            set(included ${CMAKE_MATCH_1})
            if(NOT included IN_LIST permitted)
                file(RELATIVE_PATH relative ${dir}/.. ${file})
                list(APPEND violations "${relative}: ${included} (모듈 ${module} 의 DEPS 에 없음)")
            endif()
        endforeach()
    endforeach()
endforeach()

if(violations)
    list(JOIN violations "\n  " text)
    message(FATAL_ERROR "선언하지 않은 모듈을 include 한다:\n  ${text}")
endif()
