#include "argus/control/guidance/include/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace follow {
PidController::PidController(PidGains gains, double low, double high, bool anti_windup,
                             double back_calculation)
    : gains_(gains),
      low_(low),
      high_(high),
      back_calculation_(back_calculation),
      anti_windup_(anti_windup) {
    if (!std::isfinite(gains.p + gains.i + gains.d + low + high + back_calculation) ||
        gains.p < 0 || gains.i < 0 || gains.d < 0 || low >= high || back_calculation < 0 ||
        back_calculation > 1) {
        throw std::invalid_argument("invalid PID gains or limits");
    }
}
void PidController::reset() {
    previous_measurement_.reset();
    terms_ = {};
}
double PidController::update(double measurement, double setpoint, double dt_s, bool integrate) {
    if (!std::isfinite(measurement + setpoint + dt_s) || dt_s <= 0) {
        throw std::invalid_argument("invalid PID sample or interval");
    }
    // simple-pid의 기본 sample_time은10ms. 제어 호출은 정상적으로50ms다.
    if (previous_measurement_ && dt_s < .01) {
        return terms_.output;
    }
    const double last_output = terms_.output;
    terms_.p = gains_.p * (setpoint - measurement);
    terms_.i = integrate
                   ? std::clamp(terms_.i + gains_.i * (setpoint - measurement) * dt_s, low_, high_)
                   : 0.;
    terms_.d =
        previous_measurement_ ? -gains_.d * (measurement - *previous_measurement_) / dt_s : 0.;
    terms_.output = std::clamp(terms_.p + terms_.i + terms_.d, low_, high_);
    previous_measurement_ = measurement;
    // 원본 CustomPID는 제한된 출력의 변화량으로 적분을 back-calculate한다.
    if (integrate && anti_windup_ && terms_.output != last_output &&
        (terms_.output >= high_ || terms_.output <= low_)) {
        terms_.i -= (terms_.output - last_output) * back_calculation_;
    }
    return terms_.output;
}
}  // namespace follow
