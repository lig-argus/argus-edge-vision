#pragma once
namespace follow {
struct YawSmoothingConfig {
    bool enabled{true};
    double deadzone_deg_s{.5}, maximum_change_deg_s2{90}, alpha{.7};
    bool speed_scaling{true};
    double minimum_speed_m_s{.5}, maximum_speed_m_s{5}, low_speed_factor{.5};
};
class YawRateSmoother final {
public:
    explicit YawRateSmoother(YawSmoothingConfig config = {});
    double update(double yaw_deg_s, double dt_s, double forward_speed_m_s);
    void reset();

private:
    YawSmoothingConfig config_;
    double previous_limited_{0}, filtered_{0};
};
}  // namespace follow
