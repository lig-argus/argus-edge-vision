#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "argus/control/contracts/include/control_intent.hpp"
#include "argus/control/guidance/include/pid_controller.hpp"
#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/vehicle/contracts/include/vehicle_state.hpp"
namespace follow {
struct ImageCenterConfig {
    PidGains forward{.8, .02, .35}, yaw{12., .5, 1.5}, altitude{2., .03, .05};
    double width{640}, height{480}, fx{946.65}, deadzone{.05};
    double max_forward{1.}, max_backward{.8}, max_vertical{.7}, max_yaw{75.};
    double acceleration{.6}, braking{1.8}, yaw_acceleration{300.};
    double ground_down{0}, target_height{4.}, min_height{3.5}, max_height{4.5};
};
// Explicit outdoor image servo. No target-velocity prediction or metric standoff command.
class ImageCenterFollower final {
    struct Axis {
        PidGains gains;
        double integral{0};
        std::optional<double> previous;
        PidTerms terms{};
        void reset() {
            integral = 0;
            previous.reset();
            terms = {};
        }
        double update(double error, double dt, double low, double high, bool integrate = true) {
            const double derivative = previous ? gains.d * (error - *previous) / dt : 0.;
            const double candidate =
                integrate ? std::clamp(integral + gains.i * error * dt, low, high) : integral;
            const double raw = gains.p * error + candidate + derivative;
            const double bounded = std::clamp(raw, low, high);
            if (integrate && (raw - bounded) * error <= 0) {
                integral = candidate;
            }
            terms = {gains.p * error, integral, derivative,
                     std::clamp(gains.p * error + integral + derivative, low, high)};
            previous = error;
            return terms.output;
        }
    };
    ImageCenterConfig c_;
    Axis forward_, yaw_, altitude_;
    std::optional<FollowCommand> previous_;
    static double zone(double x, double radius) {
        return std::copysign(std::max(0., std::abs(x) - radius), x);
    }
    static bool good_gains(PidGains g) {
        return std::isfinite(g.p + g.i + g.d) && g.p >= 0 && g.i >= 0 && g.d >= 0;
    }

public:
    explicit ImageCenterFollower(ImageCenterConfig c)
        : c_(c), forward_{c.forward}, yaw_{c.yaw}, altitude_{c.altitude} {
        if (!std::isfinite(c.width + c.height + c.fx + c.deadzone + c.max_forward + c.max_backward +
                           c.max_vertical + c.max_yaw + c.acceleration + c.braking +
                           c.yaw_acceleration + c.ground_down + c.target_height + c.min_height +
                           c.max_height) ||
            c.width <= 0 || c.height <= 0 || c.fx <= 0 || c.deadzone <= 0 || c.deadzone > .2 ||
            c.max_forward <= 0 || c.max_backward <= 0 || c.max_vertical <= 0 || c.max_yaw <= 0 ||
            c.acceleration <= 0 || c.braking < c.acceleration || c.yaw_acceleration <= 0 ||
            c.min_height < 1 || c.max_height > 4.8 || c.target_height <= c.min_height ||
            c.target_height >= c.max_height || !good_gains(c.forward) || !good_gains(c.yaw) ||
            !good_gains(c.altitude)) {
            throw std::invalid_argument("invalid outdoor image PID configuration");
        }
    }
    void reset() {
        forward_.reset();
        yaw_.reset();
        altitude_.reset();
        previous_.reset();
    }
    std::optional<FollowCommand> update(const Pose& p, const TrackRecordV1& box, double dt) {
        constexpr double rad = .017453292519943295;
        if (!p.have_position || !p.have_attitude || !p.have_velocity || dt <= 0 || dt > .2 ||
            !std::isfinite(p.down_m + p.yaw_deg + p.velocity_north_m_s + p.velocity_east_m_s +
                           p.velocity_down_m_s + dt + box.x1 + box.x2 + box.y1 + box.y2) ||
            box.x1 < 0 || box.x2 > c_.width || box.y1 < 0 || box.y2 > c_.height ||
            box.x2 <= box.x1 || box.y2 <= box.y1) {
            return {};
        }
        const double x = (box.x1 + box.x2) / c_.width - 1., y = (box.y1 + box.y2) / c_.height - 1.;
        const double ex = std::atan(zone(x, c_.deadzone) * c_.width / (2 * c_.fx));
        const double ey = -zone(y, c_.deadzone);
        const double height = c_.ground_down - p.down_m;
        const double ez = (c_.ground_down - c_.target_height) - p.down_m;
        if (std::abs(x) <= c_.deadzone) {
            yaw_.reset();
        }
        if (std::abs(y) <= c_.deadzone) {
            forward_.reset();
        }
        auto f = forward_, w = yaw_, z = altitude_;
        FollowCommand u;
        u.vx_m_s = ey == 0 ? 0 : f.update(ey, dt, -c_.max_backward, c_.max_forward);
        u.yaw_rate_deg_s =
            ex == 0 ? 0 : w.update(ex, dt, -c_.max_yaw * rad, c_.max_yaw * rad) / rad;
        u.vz_m_s = z.update(ez, dt, -c_.max_vertical, c_.max_vertical);
        const double heading = p.yaw_deg * rad;
        const double current =
            p.velocity_north_m_s * std::cos(heading) + p.velocity_east_m_s * std::sin(heading);
        const double start = previous_ ? previous_->vx_m_s : current;
        const double budget =
            start * u.vx_m_s < 0 || (start * (u.vx_m_s - start) < 0) ? c_.braking : c_.acceleration;
        const double limited = std::clamp(u.vx_m_s, start - budget * dt, start + budget * dt);
        if ((u.vx_m_s - limited) * ey > 0) {
            f = forward_, f.update(ey, dt, -c_.max_backward, c_.max_forward, false);
        }
        u.vx_m_s = std::clamp(limited, -c_.max_backward, c_.max_forward);
        const double lastyaw = previous_ ? previous_->yaw_rate_deg_s : 0.;
        const double limited_yaw = std::clamp(u.yaw_rate_deg_s, lastyaw - c_.yaw_acceleration * dt,
                                              lastyaw + c_.yaw_acceleration * dt);
        if ((u.yaw_rate_deg_s - limited_yaw) * ex > 0) {
            w = yaw_, w.update(ex, dt, -c_.max_yaw * rad, c_.max_yaw * rad, false);
        }
        u.yaw_rate_deg_s = limited_yaw;
        const double lastz = previous_ ? previous_->vz_m_s : p.velocity_down_m_s;
        const double limited_z =
            std::clamp(u.vz_m_s, lastz - c_.acceleration * dt, lastz + c_.acceleration * dt);
        const bool floor = height - std::max(0., p.velocity_down_m_s) / 2 <= c_.min_height;
        const bool ceiling = height + std::max(0., -p.velocity_down_m_s) / 2 >= c_.max_height;
        const double safe_z = floor     ? std::min(0., limited_z)
                              : ceiling ? std::max(0., limited_z)
                                        : limited_z;
        if ((u.vz_m_s - safe_z) * ez > 0) {
            z = altitude_, z.update(ez, dt, -c_.max_vertical, c_.max_vertical, false);
        }
        u.vz_m_s = std::clamp(safe_z, -c_.max_vertical, c_.max_vertical);
        forward_ = f;
        yaw_ = w;
        altitude_ = z;
        previous_ = u;
        return u;
    }
    std::array<PidTerms, 4> terms() const {
        return {forward_.terms, PidTerms{}, yaw_.terms, altitude_.terms};
    }
};
}  // namespace follow
