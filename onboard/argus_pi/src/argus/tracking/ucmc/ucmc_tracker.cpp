#include "argus/tracking/ucmc/include/ucmc_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "lapjv.h"
namespace follow {
UcmcTracker::UcmcTracker(UcmcParameters config) : config_(config), dt_(0) {
    if (!std::isfinite(config.a1) || !std::isfinite(config.a2) || config.a1 <= 0 ||
        config.a2 <= 0 || !std::isfinite(config.wx) || !std::isfinite(config.wy) || config.wx < 0 ||
        config.wy < 0 || !std::isfinite(config.vmax) || config.vmax <= 0 ||
        !std::isfinite(config.high_score) || config.high_score < 0 || config.high_score > 1 ||
        config.max_age <= 0 || config.fps <= 0) {
        throw std::invalid_argument("invalid UCMC parameters");
    }
    dt_ = 1. / config.fps;
}
void UcmcTracker::set_dt(double dt) {
    if (!std::isfinite(dt) || dt < 0) {
        throw std::invalid_argument("invalid UCMC sensor dt");
    }
    dt_ = dt;
    for (auto& track : tracks_) {
        track.set_dt(dt, config_.wx, config_.wy);
    }
}
void UcmcTracker::predict_no_frame() {
    for (auto& track : tracks_) {
        track.predict();
    }
}
UcmcTracker::Assignment UcmcTracker::associate(const std::vector<MetricBoxObservation>& detections,
                                               const std::vector<std::size_t>& rows,
                                               const std::vector<std::size_t>& cols,
                                               double threshold) const {
    Assignment out;
    if (rows.empty() || cols.empty()) {
        for (std::size_t i = 0; i < rows.size(); ++i) {
            out.rows.push_back(i);
        }
        for (std::size_t j = 0; j < cols.size(); ++j) {
            out.columns.push_back(j);
        }
        return out;
    }
    // Python lapjv(extend_cost=True,cost_limit=...)과 같은 dummy 비용 구성이다.
    std::size_t n = rows.size() + cols.size();
    if (n > 4096) {
        throw std::runtime_error("UCMC assignment matrix too large");
    }
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, threshold / 2.));
    for (std::size_t i = 0; i < rows.size(); ++i) {
        for (std::size_t j = 0; j < cols.size(); ++j) {
            matrix[i][j] = tracks_.at(cols[j]).distance(detections.at(rows[i]));
        }
    }
    for (std::size_t i = rows.size(); i < n; ++i) {
        for (std::size_t j = cols.size(); j < n; ++j) {
            matrix[i][j] = 0;
        }
    }
    std::vector<double*> pointers(n);
    for (std::size_t i = 0; i < n; ++i) {
        pointers[i] = matrix[i].data();
    }
    std::vector<int_t> x(n), y(n);
    if (lapjv_internal(static_cast<uint_t>(n), pointers.data(), x.data(), y.data()) != 0) {
        throw std::runtime_error("UCMC LAPJV assignment failed");
    }
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (x[i] >= 0 && static_cast<std::size_t>(x[i]) < cols.size()) {
            out.matches.emplace_back(i, x[i]);
        } else {
            out.rows.push_back(i);
        }
    }
    for (std::size_t j = 0; j < cols.size(); ++j) {
        if (y[j] < 0 || static_cast<std::size_t>(y[j]) >= rows.size()) {
            out.columns.push_back(j);
        }
    }
    return out;
}
std::vector<UcmcAssociation> UcmcTracker::update(
    const std::vector<MetricBoxObservation>& detections) {
    std::vector<std::size_t> high, low;
    for (std::size_t i = 0; i < detections.size(); ++i) {
        (detections[i].box[4] >= config_.high_score ? high : low).push_back(i);
    }
    for (auto& track : tracks_) {
        track.predict();
        track.detidx = -1;
    }
    std::vector<std::size_t> mature = confirmed_;
    mature.insert(mature.end(), coasted_.begin(), coasted_.end());
    auto first = associate(detections, high, mature, config_.a1);
    std::vector<std::size_t> remaining_detections, remaining_tracks;
    auto match = [&](std::size_t detection, std::size_t track, bool tentative) {
        auto& t = tracks_.at(track);
        t.update(detections.at(detection));
        t.death_count = 0;
        t.detidx = static_cast<int>(detection);
        if (tentative) {
            if (++t.birth_count >= 2) {
                t.birth_count = 0;
                t.status = UcmcTrackStatus::Confirmed;
            }
        } else {
            t.status = UcmcTrackStatus::Confirmed;
        }
    };
    for (auto [row, col] : first.matches) {
        match(high[row], mature[col], false);
    }
    for (auto row : first.rows) {
        remaining_detections.push_back(high[row]);
    }
    for (auto col : first.columns) {
        remaining_tracks.push_back(mature[col]);
    }
    // 공식 구현은 low나 remain이 비어 있으면 이 단계에서 Coasted로 바꾸지 않는다.
    if (!low.empty() && !remaining_tracks.empty()) {
        auto second = associate(detections, low, remaining_tracks, config_.a2);
        for (auto col : second.columns) {
            auto& t = tracks_.at(remaining_tracks[col]);
            t.status = UcmcTrackStatus::Coasted;
            t.detidx = -1;
        }
        for (auto [row, col] : second.matches) {
            match(low[row], remaining_tracks[col], false);
        }
    }
    auto third = associate(detections, remaining_detections, tentative_, config_.a1);
    for (auto [row, col] : third.matches) {
        match(remaining_detections[row], tentative_[col], true);
    }
    for (auto col : third.columns) {
        tracks_.at(tentative_[col]).detidx = -1;
    }
    for (auto row : third.rows) {
        std::size_t index = remaining_detections[row];
        tracks_.emplace_back(detections[index], config_.wx, config_.wy, config_.vmax, dt_);
        tracks_.back().detidx = static_cast<int>(index);
    }
    for (auto& track : tracks_) {
        ++track.death_count;
    }
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [&](const auto& t) {
                           return (t.status == UcmcTrackStatus::Coasted &&
                                   t.death_count >= config_.max_age) ||
                                  (t.status == UcmcTrackStatus::Tentative && t.death_count >= 2);
                       }),
        tracks_.end());
    confirmed_.clear();
    coasted_.clear();
    tentative_.clear();
    std::vector<UcmcAssociation> result;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        auto& t = tracks_[i];
        if (t.detidx >= 0 && static_cast<std::size_t>(t.detidx) < detections.size()) {
            const auto& b = detections[t.detidx].box;
            t.width = b[2] - b[0];
            t.height = b[3] - b[1];
        }
        if (t.status == UcmcTrackStatus::Confirmed) {
            confirmed_.push_back(i);
            if (t.detidx >= 0) {
                result.push_back({t.id, static_cast<std::size_t>(t.detidx)});
            }
        } else if (t.status == UcmcTrackStatus::Coasted) {
            coasted_.push_back(i);
        } else {
            tentative_.push_back(i);
        }
    }
    return result;
}
}  // namespace follow
