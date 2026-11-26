#pragma once
#include <optional>

namespace follow {
struct PidGains {
    double p{0}, i{0}, d{0};
};
struct PidTerms {
    double p{0}, i{0}, d{0}, output{0};
};

// PixEagle CustomPID + simple-pid2.0.1의 기본 측정 미분/적분 제한 동작.
class PidController final {
public:
    PidController(PidGains gains, double low, double high, bool anti_windup = true,
                  double back_calculation = .1);
    double update(double measurement, double setpoint, double dt_s, bool integrate = true);
    void reset();
    const PidTerms& terms() const { return terms_; }

private:
    PidGains gains_;
    double low_, high_, back_calculation_;
    bool anti_windup_;
    std::optional<double> previous_measurement_;
    PidTerms terms_;
};
}  // namespace follow
