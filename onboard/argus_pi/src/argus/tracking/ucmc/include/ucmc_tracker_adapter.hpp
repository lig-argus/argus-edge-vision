// 수정 trackers.py의 센서 dt와 timer 예측을 C++ UCMC에 연결한다. 비교 후보 분기는 없다.
#pragma once
#include <optional>

#include "argus/tracking/ucmc/include/ucmc_tracker.hpp"
namespace follow {
class UcmcTrackerAdapter {
public:
    explicit UcmcTrackerAdapter(UcmcParameters p = {}) : core_(p) {}
    std::vector<UcmcAssociation> update(const std::vector<MetricBoxObservation>&, double sensor_s);
    void predict_no_frame(double sensor_s);
    const std::vector<UcmcKalmanTrack>& diagnostic_tracks() const { return core_.tracks(); }
    std::uint64_t timer_predictions() const { return timer_predictions_; }

private:
    UcmcTracker core_;
    std::optional<double> prediction_s_, observation_s_;
    std::uint64_t timer_predictions_{0};
};
}  // namespace follow
