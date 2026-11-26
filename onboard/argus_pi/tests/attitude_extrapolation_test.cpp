// 2026-11-16 FC 자세 외삽(extrapolate_attitude) 시험.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/perception/geometry/include/attitude_interpolation.hpp"

using follow::TimedAttitudeSample;

namespace {
int failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::cerr << "실패: " << what << '\n';
    }
}
double angle_deg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const auto qa = follow::attitude_rotation(a(0), a(1), a(2));
    const auto qb = follow::attitude_rotation(b(0), b(1), b(2));
    return qa.angularDistance(qb) * 180. / 3.141592653589793;
}
// 일정한 몸체 각속도 w로 도는 자세: R(t) = R0·exp([w]× t)
Eigen::Quaterniond rotate(const Eigen::Quaterniond& start, const Eigen::Vector3d& w, double t) {
    const double angle = w.norm() * t;
    return angle > 0 ? start * Eigen::Quaterniond(Eigen::AngleAxisd(angle, w.normalized())) : start;
}
}  // namespace

int main() {
    const std::int64_t t0 = 10'000'000'000;
    const std::int64_t ms = 1'000'000;
    const Eigen::Quaterniond r0 = follow::attitude_rotation(0.05, -0.08, 1.2);
    const double limit = 40. * 3.141592653589793 / 180.;
    for (const Eigen::Vector3d& w : {Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0., 0., 0.6),
                                     Eigen::Vector3d(0.3, 0.2, -0.1), Eigen::Vector3d(0, 0, 0)}) {
        TimedAttitudeSample a{t0, t0 + 2 * ms, r0, 0, w};
        TimedAttitudeSample b{t0 + 40 * ms, t0 + 42 * ms, rotate(r0, w, 0.040), 1, w};
        const std::int64_t now = t0 + 60 * ms;
        for (const std::int64_t dt : {1 * ms, 10 * ms, 20 * ms, 25 * ms}) {
            const auto both = follow::interpolate_attitude({a, b}, t0 + dt, now);
            const auto only = follow::extrapolate_attitude({a}, t0 + dt, now, 25 * ms, limit);
            check(both.has_value() && only.has_value(), "보간·외삽 결과 있음");
            if (both && only) {
                // 일정 각속도에서 slerp(R0, R0·exp(w·40ms), dt/40ms) = R0·exp(w·dt)
                check(angle_deg(both->rpy, only->rpy) < 1e-6,
                      "일정 각속도 보간=외삽 dt=" + std::to_string(dt / ms));
                check(only->extrapolated_ns == dt && only->endpoint_delta_ns == dt,
                      "외삽 시간 기록");
                check(both->extrapolated_ns == 0, "보간은 외삽 0");
            }
        }
    }
    const Eigen::Vector3d w(0.1, 0.0, 0.2);
    TimedAttitudeSample a{t0, t0 + 2 * ms, r0, 0, w};
    TimedAttitudeSample later{t0 + 30 * ms, t0 + 32 * ms, rotate(r0, w, 0.03), 1, w};
    const std::int64_t now = t0 + 50 * ms;
    check(!follow::extrapolate_attitude({a}, t0 + 10 * ms, now, 0, limit), "꺼짐(0ms)");
    check(!follow::extrapolate_attitude({a, later}, t0 + 10 * ms, now, 25 * ms, limit),
          "뒤 표본이 있으면 외삽 안 함");
    check(!follow::extrapolate_attitude({a}, t0 + 26 * ms, now, 25 * ms, limit), "상한 시간 초과");
    TimedAttitudeSample fast{t0, t0 + 2 * ms, r0, 0,
                             Eigen::Vector3d(0, 0, 41. * 3.14159265 / 180.)};
    check(!follow::extrapolate_attitude({fast}, t0 + 10 * ms, now, 25 * ms, limit),
          "각속도 상한 초과");
    TimedAttitudeSample no_rate{t0, t0 + 2 * ms, r0, 0};
    check(!follow::extrapolate_attitude({no_rate}, t0 + 10 * ms, now, 25 * ms, limit),
          "각속도 없음");
    TimedAttitudeSample nan_rate{t0, t0 + 2 * ms, r0, 0, Eigen::Vector3d(NAN, 0, 0)};
    check(!follow::extrapolate_attitude({nan_rate}, t0 + 10 * ms, now, 25 * ms, limit),
          "각속도 NaN");
    check(!follow::extrapolate_attitude({a}, t0 + 10 * ms, t0 + 5 * ms, 25 * ms, limit),
          "미래 목표");
    TimedAttitudeSample old{t0, t0 + 2 * ms, r0, 0, w};
    check(!follow::extrapolate_attitude({old}, t0 + 10 * ms, t0 + 300 * ms, 25 * ms, limit),
          "200ms 넘은 오래된 표본");
    check(follow::extrapolate_attitude({a}, t0, now, 25 * ms, limit).has_value(),
          "Δt 0 (같은 시각)");
    if (failures != 0) {
        std::cerr << failures << "개 실패\n";
        return 1;
    }
    std::cout << "AttitudeExtrapolationTest 통과\n";
    return 0;
}
