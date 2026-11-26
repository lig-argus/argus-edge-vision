#include "argus/tracking/ucmc/include/ucmc_tracker_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace follow {
std::vector<UcmcAssociation> UcmcTrackerAdapter::update(
    const std::vector<MetricBoxObservation>& observations, double sensor_s) {
    if (!std::isfinite(sensor_s)) {
        throw std::invalid_argument("invalid tracking sensor time");
    }
    double effective = prediction_s_ ? std::max(sensor_s, *prediction_s_) : sensor_s;
    double dt = prediction_s_ ? effective - *prediction_s_ : 1. / 30.;
    core_.set_dt(std::max(0., dt));
    prediction_s_ = effective;
    observation_s_ = sensor_s;
    return core_.update(observations);
}
void UcmcTrackerAdapter::predict_no_frame(double sensor_s) {
    if (!prediction_s_ || !observation_s_ || sensor_s - *observation_s_ < .25 ||
        sensor_s <= *prediction_s_) {
        return;
    }
    core_.set_dt(sensor_s - *prediction_s_);
    core_.predict_no_frame();
    prediction_s_ = sensor_s;
    ++timer_predictions_;
}
}  // namespace follow
