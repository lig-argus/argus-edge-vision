// 시험용 수평 거리와 유효성·품질·사유.
#pragma once

#include <limits>

namespace follow {

// rho_m은 수평 거리(m)다. valid가 false이면 NaN과 사유를 확인하고 제어 입력으로 쓰지 않는다.
struct RangeEstimate {
    double rho_m{std::numeric_limits<double>::quiet_NaN()};
    const char* quality{"INVALID"};
    const char* reason{"not_calculated"};
    bool valid{false};
};

}  // namespace follow
