#include "argus/diagnostics/tracking/include/target_view_codec.hpp"

#include <stdexcept>

#include "argus/common/serialization/include/message_pack.hpp"
namespace follow {
namespace {
template <class T>
void optional_field(msgpack::packer<msgpack::sbuffer>& p, const char* key,
                    const std::optional<T>& value) {
    p.pack(key);
    if (value) {
        p.pack(*value);
    } else {
        p.pack_nil();
    }
}
const char* selection_name(TargetSelectionSnapshot::Phase phase) {
    if (phase == TargetSelectionSnapshot::Phase::Pending) {
        return "PENDING";
    }
    if (phase == TargetSelectionSnapshot::Phase::Following) {
        return "MONITORING";
    }
    return "UNSELECTED";
}
}  // namespace
std::string encode_target_view(const TargetViewSnapshot& view) {
    const auto& s = view.selection;
    if (!view.objects.empty() && (!s.latest_frame || !view.frame_available)) {
        throw std::invalid_argument("target object view requires a current frame");
    }
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    p.pack_map(15);
    msgpack_pack_field(p, "schema", "argus.tracking.target_view.v1");
    msgpack_pack_field(p, "flight_control", false);
    msgpack_pack_field(
        p, "published_monotonic_ns",
        std::chrono::duration_cast<std::chrono::nanoseconds>(view.published_at.time_since_epoch())
            .count());
    msgpack_pack_field(p, "command_epoch", s.command_epoch);
    msgpack_pack_field(p, "target_revision", s.target_revision);
    msgpack_pack_field(p, "selection_state", selection_name(s.phase));
    msgpack_pack_field(p, "target_phase", target_phase_name(s.target_phase));
    optional_field(
        p, "target_id",
        s.mission_target_id > 0 ? std::optional<std::uint64_t>(s.mission_target_id) : std::nullopt);
    optional_field(p, "binding_generation",
                   s.target ? std::optional<std::uint64_t>(s.target->generation) : std::nullopt);
    optional_field(p, "binding_tracker_id",
                   s.target ? std::optional<std::int64_t>(s.target->track_id) : std::nullopt);
    msgpack_pack_field(p, "frame_available", view.frame_available);
    optional_field(p, "source_age_s", view.source_age_s);
    msgpack_pack_field(p, "age_since_real_s", s.age_since_real_s);
    p.pack("frame_key");
    if (!s.latest_frame) {
        p.pack_nil();
    } else {
        const auto& f = *s.latest_frame;
        p.pack_map(11);
        msgpack_pack_field(p, "source", f.source);
        msgpack_pack_field(p, "generation", f.generation);
        msgpack_pack_field(p, "frame_id", f.frame_id);
        msgpack_pack_field(p, "sample_monotonic_ns", f.sample_monotonic_ns);
        msgpack_pack_field(p, "image_width", f.image_width);
        msgpack_pack_field(p, "image_height", f.image_height);
        msgpack_pack_field(p, "coordinate_frame", f.coordinate_frame);
        msgpack_pack_field(p, "clock_domain", f.clock_domain);
        msgpack_pack_field(p, "timestamp_source", f.timestamp_source);
        msgpack_pack_field(p, "exposure_time_valid", f.exposure_time_valid);
        optional_field(p, "exposure_monotonic_ns", f.exposure_monotonic_ns);
    }
    p.pack("objects");
    p.pack_array(view.objects.size());
    for (const auto& identity : view.objects) {
        const auto& track = s.latest_frame->tracks.at(identity.track_index);
        p.pack_map(11);
        msgpack_pack_field(p, "tracker_id", track.track_id);
        optional_field(p, "target_id", identity.target_id);
        msgpack_pack_field(p, "class_id", track.class_id);
        msgpack_pack_field(p, "label", track.label);
        msgpack_pack_field(p, "confidence", track.confidence);
        p.pack("bbox");
        p.pack_array(4);
        p.pack(track.x1);
        p.pack(track.y1);
        p.pack(track.x2);
        p.pack(track.y2);
        msgpack_pack_field(p, "track_state", track.state);
        msgpack_pack_field(p, "real_detection", track.real_detection);
        msgpack_pack_field(p, "last_real_detection_monotonic_ns",
                           track.last_real_detection_monotonic_ns);
        msgpack_pack_field(p, "is_selected", identity.is_selected);
        msgpack_pack_field(p, "binding_confirmed", identity.binding_confirmed);
    }
    return {buffer.data(), buffer.size()};
}
}  // namespace follow
