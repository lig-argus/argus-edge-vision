#include <cassert>
#include <chrono>
#include <iostream>

#include "argus/control/target/include/target_reacquirer.hpp"

using namespace follow;

namespace {
TrackRecordV1 vehicle(std::int64_t id, std::int64_t stamp, bool expanded) {
    TrackRecordV1 track;
    track.track_id = id;
    track.class_id = 2;
    track.label = "car";
    track.confidence = .9;
    track.state = "CONFIRMED";
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = stamp;
    track.x1 = 104;
    track.y1 = expanded ? 195 : 204;
    track.x2 = expanded ? 130 : 119;
    track.y2 = expanded ? 263 : 255;
    track.ground = std::array<double, 6>{2., 0., .05, 0., .05, 0.};
    return track;
}

void same_selected_id_growth_is_bounded() {
    for (int kind = 0; kind < 11; ++kind) {
        TargetRecoveryConfig config;
        config.prefer_selected_continuity = kind != 1;
        config.ground_contact_anchor = true;
        TargetReacquirer reacquirer(config);
        TargetMemory memory;
        memory.binding = {1, 1};
        memory.last_real_sample_ns = 1000000000;
        memory.last_real_track = vehicle(1, memory.last_real_sample_ns, false);
        TracksV1 frame;
        frame.schema = kTracksV2Schema;
        frame.generation = kind == 3 ? 2 : 1;
        frame.sample_monotonic_ns = 1050000000;
        frame.image_width = 640;
        frame.image_height = 480;
        auto track = vehicle(kind == 2 ? 2 : 1, frame.sample_monotonic_ns, true);
        if (kind == 4) {
            track.x2 = track.x1 + 100;
            track.y2 = track.y1 + 100;
        }
        if (kind == 5) {
            (*track.ground)[0] = 5.;
        }
        if (kind == 6) {
            track.confidence = .2;
        }
        if (kind == 7) {
            track.state = "PREDICTED";
            track.real_detection = false;
            track.last_real_detection_monotonic_ns = memory.last_real_sample_ns;
        }
        if (kind == 8) {
            track.x2 = track.x1 + 6;
            track.y2 = track.y1 + 20;
        }
        if (kind == 9) {
            track.label = "different_vehicle";
        }
        if (kind == 10) {
            track.last_real_detection_monotonic_ns = memory.last_real_sample_ns;
        }
        assert(valid_track_record(track));
        frame.tracks = {track};
        const auto result = reacquirer.find_candidate(memory, frame, &memory.last_real_track);
        assert(kind == 0 ? result.kind == ReacquisitionResult::Kind::Unique
                         : result.kind == ReacquisitionResult::Kind::None);
        assert(memory.last_real_sample_ns == 1000000000);
        assert(memory.last_real_track.x2 == 119);
    }
}

void competing_ground_candidate_remains_ambiguous() {
    TargetRecoveryConfig config;
    config.prefer_selected_continuity = true;
    config.ground_contact_anchor = true;
    TargetReacquirer reacquirer(config);
    TargetMemory memory;
    memory.binding = {1, 1};
    memory.last_real_sample_ns = 1000000000;
    memory.last_real_track = vehicle(1, memory.last_real_sample_ns, false);
    TracksV1 frame;
    frame.schema = kTracksV2Schema;
    frame.generation = 1;
    frame.sample_monotonic_ns = 1050000000;
    frame.image_width = 640;
    frame.image_height = 480;
    frame.tracks = {vehicle(1, frame.sample_monotonic_ns, true),
                    vehicle(2, frame.sample_monotonic_ns, false)};
    assert(reacquirer.find_candidate(memory, frame).kind == ReacquisitionResult::Kind::Ambiguous);
}
}  // namespace

int main() {
    same_selected_id_growth_is_bounded();
    competing_ground_candidate_remains_ambiguous();
    std::cout << "2.31x same-ID growth admitted; excessive growth, shrink, wrong ID/generation, "
                 "position jump, low confidence, predicted input and ambiguity guarded\n";
}
