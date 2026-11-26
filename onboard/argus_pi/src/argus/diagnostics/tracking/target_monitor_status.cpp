#include "argus/diagnostics/tracking/include/target_monitor_status.hpp"

#include "argus/common/serialization/include/message_pack.hpp"
namespace follow {
std::string encode_target_monitor_status(const TargetSelectionSnapshot& s, Clock::time_point now) {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    p.pack_map(15);
    msgpack_pack_field(p, "schema", "argus.tracking.target_monitor.v1");
    msgpack_pack_field(p, "flight_control", false);
    msgpack_pack_field(p, "monitoring", s.phase == TargetSelectionSnapshot::Phase::Following);
    msgpack_pack_field(
        p, "monotonic_ns",
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    msgpack_pack_field(p, "mission_target_id", s.mission_target_id);
    msgpack_pack_field(p, "command_epoch", s.command_epoch);
    msgpack_pack_field(p, "target_phase", target_phase_name(s.target_phase));
    msgpack_pack_field(p, "reason", s.recovery_reason);
    msgpack_pack_field(p, "target_revision", s.target_revision);
    msgpack_pack_field(p, "generation", s.target ? s.target->generation : 0);
    msgpack_pack_field(p, "track_id", s.target ? s.target->track_id : 0);
    msgpack_pack_field(p, "fresh_real_detection", s.fresh_real_detection);
    msgpack_pack_field(p, "age_since_real_s", s.age_since_real_s);
    msgpack_pack_field(p, "sample_monotonic_ns",
                       s.latest_frame ? s.latest_frame->sample_monotonic_ns : 0);
    msgpack_pack_field(p, "last_real_detection_monotonic_ns",
                       s.selected_track ? s.selected_track->last_real_detection_monotonic_ns : 0);
    return {buffer.data(), buffer.size()};
}
}  // namespace follow
