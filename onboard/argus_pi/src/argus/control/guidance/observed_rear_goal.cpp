#include "argus/control/guidance/include/observed_rear_goal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace follow {
ObservedRearGoal::ObservedRearGoal(double distance, double spacing)
    : distance_m_(distance), sample_spacing_m_(spacing) {
    if (!std::isfinite(distance) || distance <= 0 || distance > 50 || !std::isfinite(spacing) ||
        spacing <= 0 || spacing > distance) {
        throw std::invalid_argument("invalid observed rear path configuration");
    }
}
void ObservedRearGoal::reset() {
    points_.clear();
    reversing_ = false;
}
RearGoal ObservedRearGoal::update(std::array<double, 2> point, std::array<double, 2> direction,
                                  double speed, bool moving,
                                  std::optional<std::array<double, 2>> observed_velocity,
                                  bool commit_stop_anchor) {
    if (!std::isfinite(point[0] + point[1] + speed) || speed < 0 ||
        !std::isfinite(direction[0] + direction[1])) {
        throw std::invalid_argument("invalid observed rear path input");
    }
    if (points_.empty()) {
        const double norm = std::hypot(direction[0], direction[1]);
        if (norm < 1e-9) {
            throw std::invalid_argument("missing initial rear direction");
        }
        initial_direction_ = {direction[0] / norm, direction[1] / norm};
        forward_direction_ = initial_direction_;
        points_.push_back(point);
    }
    if (points_.size() == 1 && !moving) {
        points_.front() = point;
    }
    double signed_speed = speed;
    bool rewinding = false;
    if (moving && observed_velocity) {
        const double vn = (*observed_velocity)[0], ve = (*observed_velocity)[1],
                     velocity = std::hypot(vn, ve);
        if (!std::isfinite(vn + ve)) {
            throw std::invalid_argument("invalid observed reverse velocity");
        }
        const double progress = vn * forward_direction_[0] + ve * forward_direction_[1];
        if (velocity > 1e-6 && progress < -.5 * velocity) {
            reversing_ = true;
        } else if (velocity > 1e-6 && progress > .5 * velocity) {
            reversing_ = false;
        }
        if (!reversing_ && velocity > 1e-6) {
            forward_direction_ = {vn / velocity, ve / velocity};
        }
        if (reversing_ && points_.size() == 1) {
            points_.front() = point;
            signed_speed = -speed;
            rewinding = true;
        }
    }
    if (moving && observed_velocity && reversing_ && points_.size() >= 2) {
        const double vn = (*observed_velocity)[0], ve = (*observed_velocity)[1],
                     velocity = std::hypot(vn, ve);
        if (velocity > 1e-6) {
            double best = std::numeric_limits<double>::infinity();
            std::size_t segment = 0;
            std::array<double, 2> projected{};
            for (std::size_t i = 1; i < points_.size(); ++i) {
                const auto& a = points_[i - 1];
                const auto& b = points_[i];
                const double dn = b[0] - a[0], de = b[1] - a[1], dd = dn * dn + de * de;
                if (dd < 1e-9) {
                    continue;
                }
                // 위치 회귀/정지 기준 보정으로 생긴 짧은 반대 segment를 실제 경로 방향으로 쓰지
                // 않는다.
                if (dn * vn + de * ve >= -.2 * std::sqrt(dd) * velocity) {
                    continue;
                }
                const double fraction =
                    std::clamp(((point[0] - a[0]) * dn + (point[1] - a[1]) * de) / dd, 0., 1.);
                const std::array<double, 2> candidate{a[0] + fraction * dn, a[1] + fraction * de};
                const double error = std::hypot(point[0] - candidate[0], point[1] - candidate[1]);
                if (error < best) {
                    best = error;
                    segment = i;
                    projected = candidate;
                }
            }
            if (best <= sample_spacing_m_) {
                const auto front = points_.front();
                const auto next = points_[1];
                const double fn = next[0] - front[0], fe = next[1] - front[1],
                             fl = std::hypot(fn, fe);
                if (segment == 1 && fl > 1e-6 &&
                    (point[0] - front[0]) * fn + (point[1] - front[1]) * fe < 0) {
                    initial_direction_ = {fn / fl, fe / fl};
                    points_.clear();
                    points_.push_back(point);
                    projected = point;
                    segment = 1;
                }
                points_.erase(points_.begin() + segment, points_.end());
                if (std::hypot(projected[0] - points_.back()[0], projected[1] - points_.back()[1]) >
                    1e-6) {
                    points_.push_back(projected);
                }
                point = projected;
                signed_speed = -speed;
                rewinding = true;
            }
        }
    }
    if (moving && observed_velocity && reversing_ && !rewinding) {
        signed_speed = 0;
        moving = false;
    }
    const double delta = std::hypot(point[0] - points_.back()[0], point[1] - points_.back()[1]);
    if (!rewinding && ((moving && delta >= sample_spacing_m_) ||
                       (commit_stop_anchor && points_.size() > 1 && delta > 1e-6))) {
        points_.push_back(point);
    }
    while (points_.size() > 1024) {
        points_.pop_front();
    }
    // Retain an extra standback distance for genuine reversals on the already observed path.
    double history = 0;
    auto head = point;
    for (std::size_t i = points_.size(); i > 0; --i) {
        history += std::hypot(head[0] - points_[i - 1][0], head[1] - points_[i - 1][1]);
        head = points_[i - 1];
        if (history > 2 * distance_m_) {
            for (std::size_t j = 0; j < i - 1; ++j) {
                points_.pop_front();
            }
            break;
        }
    }
    double remaining = distance_m_;
    auto current = point;
    for (std::size_t i = points_.size(); i > 0; --i) {
        const auto previous = points_[i - 1];
        const double n = current[0] - previous[0], e = current[1] - previous[1];
        const double length = std::hypot(n, e);
        if (length > 1e-9) {
            if (remaining <= length) {
                const RearGoal goal{current[0] - remaining * n / length,
                                    current[1] - remaining * e / length, signed_speed * n / length,
                                    signed_speed * e / length, false};
                return goal;
            }
            remaining -= length;
        }
        current = previous;
    }
    return {current[0] - remaining * initial_direction_[0],
            current[1] - remaining * initial_direction_[1], signed_speed * initial_direction_[0],
            signed_speed * initial_direction_[1], true};
}
}  // namespace follow
