// ucMC.py의 high→low→tentative→생성→삭제→상태 갱신 순서를 보존한다.
#pragma once
#include <vector>

#include "argus/tracking/ucmc/include/ucmc_kalman_track.hpp"
namespace follow {
struct UcmcParameters {
    double a1{100}, a2{100}, wx{5}, wy{5}, vmax{10}, high_score{.3};
    int max_age{30}, fps{30};
};
class UcmcTracker {
public:
    explicit UcmcTracker(UcmcParameters config = {});
    std::vector<UcmcAssociation> update(const std::vector<MetricBoxObservation>&);
    void set_dt(double dt);
    void predict_no_frame();
    const std::vector<UcmcKalmanTrack>& tracks() const { return tracks_; }

private:
    struct Assignment {
        std::vector<std::pair<std::size_t, std::size_t>> matches;
        std::vector<std::size_t> rows, columns;
    };
    Assignment associate(const std::vector<MetricBoxObservation>&,
                         const std::vector<std::size_t>& detections,
                         const std::vector<std::size_t>& tracks, double threshold) const;
    UcmcParameters config_;
    double dt_;
    std::vector<UcmcKalmanTrack> tracks_;
    std::vector<std::size_t> confirmed_, coasted_, tentative_;
};
}  // namespace follow
