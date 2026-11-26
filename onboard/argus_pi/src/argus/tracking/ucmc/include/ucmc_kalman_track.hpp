// UCMC 공식 kalman.py/FilterPy 수식을 Eigen으로 이식한다.
#pragma once
#include <atomic>
#include <cstdint>

#include "argus/common/identity/include/float_identity.hpp"
#include "argus/tracking/ucmc/include/metric_box_observation.hpp"
namespace follow {
enum class UcmcTrackStatus { Tentative, Confirmed, Coasted };
class UcmcKalmanTrack {
public:
    UcmcKalmanTrack(const MetricBoxObservation&, double wx, double wy, double vmax, double dt);
    void set_dt(double dt, double wx, double wy);
    void predict();
    void update(const MetricBoxObservation&);
    double distance(const MetricBoxObservation&) const;
    std::int64_t id;
    std::uint64_t age{0};
    int death_count{0}, birth_count{0}, detidx{-1};
    double width{0}, height{0};
    UcmcTrackStatus status{UcmcTrackStatus::Tentative};
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};
    Eigen::Matrix4d P{Eigen::Matrix4d::Zero()};

private:
    Eigen::Matrix4d F{Eigen::Matrix4d::Identity()}, Q{Eigen::Matrix4d::Zero()};
    static FloatTrackIdSequence next_id_;
};
}  // namespace follow
