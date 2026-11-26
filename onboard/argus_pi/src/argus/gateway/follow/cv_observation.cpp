#include "argus/gateway/follow/include/cv_observation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace argus::gateway {

namespace {

constexpr std::int64_t kMaxInt = std::numeric_limits<std::int64_t>::max();

// 원본 _integer(value, minimum): type(value) is int and minimum <= value <= 2^63-1
bool integer(const Wire& value, std::int64_t minimum = 0) {
    if (value.is_number_unsigned()) {
        return value.get<std::uint64_t>() <= static_cast<std::uint64_t>(kMaxInt) &&
               static_cast<std::int64_t>(value.get<std::uint64_t>()) >= minimum;
    }
    return value.is_number_integer() && value.get<std::int64_t>() >= minimum;
}

std::int64_t as_int(const Wire& value) {
    return value.is_number_unsigned() ? static_cast<std::int64_t>(value.get<std::uint64_t>())
                                      : value.get<std::int64_t>();
}

// 원본 _number: int/float이고 유한
bool number(const Wire& value) { return value.is_number() && std::isfinite(value.get<double>()); }

double as_double(const Wire& value) { return value.get<double>(); }

bool array_of(const Wire& value, std::size_t size) {
    return value.is_array() && value.size() == size &&
           std::all_of(value.begin(), value.end(), [](const Wire& item) { return number(item); });
}

bool nonempty_text(const Wire& value) {
    return value.is_string() && !value.get_ref<const std::string&>().empty();
}

}  // namespace

void CvObservationLedger::clear() {
    observations_.clear();
    sources_.clear();
    epoch_.reset();
}

bool CvObservationLedger::observe(const Wire& frame, std::int64_t now_ns, std::int64_t fc_epoch,
                                  bool verified_fc) {
    if (!verified_fc || !frame.is_object() || now_ns < 1) {
        return false;
    }
    static constexpr std::array<const char*, 16> kRequired{"schema",
                                                           "source",
                                                           "frame_id",
                                                           "sample_monotonic_ns",
                                                           "clock_domain",
                                                           "timestamp_source",
                                                           "exposure_monotonic_ns",
                                                           "exposure_time_valid",
                                                           "image_width",
                                                           "image_height",
                                                           "coordinate_frame",
                                                           "generation",
                                                           "tracks",
                                                           "motion_sample_ns",
                                                           "ground_pose",
                                                           "fc_epoch"};
    for (const char* key : kRequired) {
        if (!frame.contains(key)) {
            return false;
        }
    }
    const Wire& stamp = frame.at("sample_monotonic_ns");
    const Wire& generation = frame.at("generation");
    const Wire& source = frame.at("source");
    const Wire& frame_number = frame.at("frame_id");
    const Wire& width = frame.at("image_width");
    const Wire& height = frame.at("image_height");
    const Wire& tracks = frame.at("tracks");
    const Wire& exposure_valid = frame.at("exposure_time_valid");
    const Wire& exposure = frame.at("exposure_monotonic_ns");
    const Wire& fc = frame.at("fc_epoch");
    const bool bad = frame.at("schema") != "argus.tracking.tracks.v5" ||
                     frame.at("clock_domain") != "host_monotonic" ||
                     frame.at("coordinate_frame") != "camera_pixel" ||
                     !nonempty_text(frame.at("timestamp_source")) || !nonempty_text(source) ||
                     source.get_ref<const std::string&>().size() > 128 || !fc.is_number_integer() ||
                     !python_equal(fc, Wire(fc_epoch)) || !integer(stamp, 1) ||
                     as_int(stamp) > now_ns || now_ns - as_int(stamp) >= kCvWindowNs ||
                     !integer(generation, 1) || !integer(frame_number) ||
                     (frame.contains("seq") && !integer(frame.at("seq"))) ||
                     !integer(frame.at("motion_sample_ns")) || !exposure_valid.is_boolean() ||
                     (exposure_valid.get<bool>() && !python_equal(exposure, stamp)) ||
                     (!exposure_valid.get<bool>() && !exposure.is_null()) || !integer(width, 1) ||
                     !integer(height, 1) || as_int(width) > 0xffffffffLL ||
                     as_int(height) > 0xffffffffLL || !tracks.is_array() || tracks.size() > 512 ||
                     (!tracks.empty() && !array_of(frame.at("ground_pose"), 6));
    if (bad) {
        return false;
    }
    const std::int64_t stamp_ns = as_int(stamp);
    const auto image_width = static_cast<double>(as_int(width));
    const auto image_height = static_cast<double>(as_int(height));
    std::set<std::int64_t> seen;
    std::vector<std::pair<std::int64_t, CvIdentity>> real;
    for (const Wire& track : tracks) {
        if (!track.is_object()) {
            return false;
        }
        const Wire ident = wire_get(track, "track_id");
        const Wire category = wire_get(track, "class_id");
        const Wire label = wire_get(track, "label");
        const Wire state = wire_get(track, "state");
        const Wire real_detection = wire_get(track, "real_detection");
        const Wire last = wire_get(track, "last_real_detection_monotonic_ns");
        const Wire confidence = wire_get(track, "confidence");
        const std::array<Wire, 4> box{wire_get(track, "x1"), wire_get(track, "y1"),
                                      wire_get(track, "x2"), wire_get(track, "y2")};
        const Wire ground = wire_get(track, "ground");
        const Wire anchor = wire_get(track, "ground_anchor");
        if (!integer(ident, 1) || seen.count(as_int(ident)) != 0 || !integer(category) ||
            as_int(category) > 0x7fffffffLL || !nonempty_text(label) || !number(confidence) ||
            as_double(confidence) < 0 || as_double(confidence) > 1) {
            return false;
        }
        if (!std::all_of(box.begin(), box.end(), [](const Wire& item) { return number(item); })) {
            return false;
        }
        const double x1 = as_double(box.at(0));
        const double y1 = as_double(box.at(1));
        const double x2 = as_double(box.at(2));
        const double y2 = as_double(box.at(3));
        const bool confirmed = state == "CONFIRMED";
        const bool x_inside = 0 <= x1 && x1 < x2 && x2 <= image_width;
        const bool y_inside = 0 <= y1 && y1 < y2 && y2 <= image_height;
        if (!x_inside || !y_inside ||
            (state != "CONFIRMED" && state != "PREDICTED" && state != "LOST") ||
            !real_detection.is_boolean() || real_detection.get<bool>() != confirmed ||
            !integer(last) || as_int(last) > stamp_ns ||
            (real_detection.get<bool>() && as_int(last) != stamp_ns)) {
            return false;
        }
        if (!ground.is_null()) {
            if (!array_of(ground, 6)) {
                return false;
            }
            const double g2 = as_double(ground.at(2));
            const double g3 = as_double(ground.at(3));
            const double g4 = as_double(ground.at(4));
            const double g5 = as_double(ground.at(5));
            const bool scale_ok = 0 < g2 && g2 <= 4 && 0 < g4 && g4 <= 4;
            const bool confidence_ok = 0 <= g5 && g5 <= 100;
            if (!scale_ok || g2 * g4 <= g3 * g3 || !confidence_ok || !array_of(anchor, 2)) {
                return false;
            }
            const bool anchor_kind = as_double(anchor.at(0)) == 0 || as_double(anchor.at(0)) == 1;
            const bool anchor_level = 0 <= as_double(anchor.at(1)) && as_double(anchor.at(1)) <= 3;
            if (!anchor_kind || !anchor_level) {
                return false;
            }
        } else if (!anchor.is_null()) {
            return false;
        }
        seen.insert(as_int(ident));
        if (real_detection.get<bool>() && !ground.is_null()) {
            real.emplace_back(as_int(ident), CvIdentity{source.get<std::string>(), as_int(category),
                                                        label.get<std::string>()});
        }
    }
    if (epoch_ != fc_epoch) {
        clear();
        epoch_ = fc_epoch;
    }
    const SourceKey key{fc_epoch, as_int(generation), source.get<std::string>()};
    const std::int64_t number_value = as_int(frame_number);
    auto found = std::find_if(sources_.begin(), sources_.end(),
                              [&key](const auto& item) { return item.first == key; });
    if (found != sources_.end() &&
        (number_value <= found->second.first || stamp_ns <= found->second.second)) {
        return false;
    }
    if (found != sources_.end()) {
        sources_.erase(found);  // move_to_end
    }
    sources_.emplace_back(key, std::make_pair(number_value, stamp_ns));
    while (sources_.size() > 32) {
        sources_.erase(sources_.begin());
    }
    for (const auto& [ident, who] : real) {
        const Key observation{fc_epoch, as_int(generation), ident, stamp_ns};
        auto existing =
            std::find_if(observations_.begin(), observations_.end(),
                         [&observation](const auto& item) { return item.first == observation; });
        if (existing != observations_.end()) {
            existing->second = who;  // 원본 dict: 값만 바꾸고 자리는 그대로
        } else {
            observations_.emplace_back(observation, who);
        }
    }
    prune(now_ns);
    return !real.empty();
}

void CvObservationLedger::prune(std::int64_t now_ns) {
    while (!observations_.empty()) {
        const Key& oldest = observations_.front().first;
        if (observations_.size() <= capacity_ && now_ns - oldest.stamp < kCvWindowNs) {
            break;
        }
        observations_.erase(observations_.begin());
    }
}

std::optional<CvObservationLedger::Key> CvObservationLedger::lookup_key(std::int64_t fc_epoch,
                                                                        const Wire& generation,
                                                                        const Wire& track_id,
                                                                        const Wire& stamp_ns) {
    const auto exact = [](const Wire& value) -> std::optional<std::int64_t> {
        if (value.is_number_integer() && !value.is_number_unsigned()) {
            return value.get<std::int64_t>();
        }
        if (value.is_number_unsigned() &&
            value.get<std::uint64_t>() <= static_cast<std::uint64_t>(kMaxInt)) {
            return static_cast<std::int64_t>(value.get<std::uint64_t>());
        }
        if (value.is_boolean()) {
            return value.get<bool>() ? 1 : 0;
        }
        if (value.is_number_float() && std::floor(value.get<double>()) == value.get<double>() &&
            std::abs(value.get<double>()) < 9.2e18) {
            return static_cast<std::int64_t>(value.get<double>());
        }
        return std::nullopt;
    };
    const auto g = exact(generation);
    const auto t = exact(track_id);
    const auto s = exact(stamp_ns);
    if (!g || !t || !s) {
        return std::nullopt;
    }
    return Key{fc_epoch, *g, *t, *s};
}

bool CvObservationLedger::contains(std::int64_t fc_epoch, const Wire& generation,
                                   const Wire& track_id, const Wire& stamp_ns,
                                   std::int64_t now_ns) {
    if (now_ns < 1 || epoch_ != fc_epoch) {
        return false;
    }
    prune(now_ns);
    const auto key = lookup_key(fc_epoch, generation, track_id, stamp_ns);
    return key && std::any_of(observations_.begin(), observations_.end(),
                              [&key](const auto& item) { return item.first == *key; });
}

std::optional<CvIdentity> CvObservationLedger::identity(std::int64_t fc_epoch,
                                                        const Wire& generation,
                                                        const Wire& track_id, const Wire& stamp_ns,
                                                        std::int64_t now_ns) {
    if (!contains(fc_epoch, generation, track_id, stamp_ns, now_ns)) {
        return std::nullopt;
    }
    const auto key = lookup_key(fc_epoch, generation, track_id, stamp_ns);
    if (!key) {
        return std::nullopt;
    }
    for (const auto& [observation, who] : observations_) {
        if (observation == *key) {
            return who;
        }
    }
    return std::nullopt;
}

Wire CvObservationLedger::debug_state() const {
    Wire observations = Wire::array();
    for (const auto& [key, who] : observations_) {
        observations.push_back(
            {key.epoch, key.generation, key.track, key.stamp, who.source, who.category, who.label});
    }
    Wire sources = Wire::array();
    for (const auto& [key, value] : sources_) {
        sources.push_back({key.epoch, key.generation, key.source, value.first, value.second});
    }
    return {{"epoch", epoch_ ? Wire(*epoch_) : Wire()},
            {"observations", observations},
            {"sources", sources}};
}

}  // namespace argus::gateway
