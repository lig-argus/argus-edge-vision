#pragma once

// 외부 시험 라이브러리 없이 쓰는 아주 작은 시험 도구.
//   ARGUS_CHECK(조건);            조건이 거짓이면 파일:줄과 식을 출력하고 실패로 센다
//   return argus::flight::test::report("이름");  실패가 없으면 0(성공)을 반환한다

#include <iostream>

namespace argus::flight::test {

inline int& failure_count() {
    static int count = 0;
    return count;
}

inline void check(bool ok, const char* expression, const char* file, int line) {
    if (!ok) {
        ++failure_count();
        std::cerr << file << ':' << line << ": CHECK failed: " << expression << '\n';
    }
}

inline int report(const char* suite) {
    const int failures = failure_count();
    std::cout << suite << ": " << (failures == 0 ? "OK" : "FAILED") << " (" << failures
              << " failures)\n";
    return failures == 0 ? 0 : 1;
}

}  // namespace argus::flight::test

// 식 문자열(#expr)과 위치(__FILE__, __LINE__)를 함께 넘기려면 매크로가 필요하다.
#define ARGUS_CHECK(expr) \
    ::argus::flight::test::check(static_cast<bool>(expr), #expr, __FILE__, __LINE__)
