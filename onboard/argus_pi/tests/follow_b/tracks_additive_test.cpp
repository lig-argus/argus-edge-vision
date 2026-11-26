// Additive v5 metadata must not weaken required identities, time or geometry.
#include <cassert>
#include <limits>

#include "argus/common/serialization/include/message_pack.hpp"
#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"

using namespace follow;
namespace {
std::string pack(const NativeJson& value) {
    const auto bytes = NativeJson::to_msgpack(value);
    return {bytes.begin(), bytes.end()};
}
NativeJson unpack(const std::string& value) {
    return NativeJson::from_msgpack(value.begin(), value.end());
}
TracksV1 message() {
    TracksV1 m;
    m.schema = kTracksV5Schema;
    m.source = "camera";
    m.frame_id = 100;
    m.sample_monotonic_ns = 1000000;
    m.clock_domain = "host_monotonic";
    m.timestamp_source = "test_host_receive";
    m.image_width = 640;
    m.image_height = 480;
    m.generation = std::numeric_limits<std::uint64_t>::max();
    m.fc_epoch = 0;
    m.motion_sample_ns = m.sample_monotonic_ns;
    m.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    TrackRecordV1 t;
    t.track_id = 1;
    t.class_id = 2;
    t.label = "car";
    t.confidence = .9;
    t.x1 = 100;
    t.y1 = 100;
    t.x2 = 200;
    t.y2 = 200;
    t.state = "CONFIRMED";
    t.real_detection = true;
    t.last_real_detection_monotonic_ns = m.sample_monotonic_ns;
    t.ground = std::array<double, 6>{1, 2, .01, 0, .01, 5};
    t.ground_anchor = std::array<double, 2>{1, 0};
    m.tracks.push_back(t);
    return m;
}
std::string duplicate_root(const std::string& input, const std::string& field) {
    auto parsed = msgpack::unpack(input.data(), input.size());
    const auto& root = parsed.get();
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> writer(buffer);
    writer.pack_map(root.via.map.size + 1);
    for (unsigned i = 0; i < root.via.map.size; ++i) {
        writer.pack(root.via.map.ptr[i].key);
        writer.pack(root.via.map.ptr[i].val);
    }
    writer.pack(field);
    writer.pack(1);
    return {buffer.data(), buffer.size()};
}
}  // namespace
int main() {
    auto original = message();
    auto sequenced = original;
    sequenced.seq = std::uint64_t(9007199254740993ULL);
    auto sequence_roundtrip = TracksV1Codec::decode(TracksV1Codec::encode(sequenced));
    assert(sequence_roundtrip.valid && sequence_roundtrip.message.seq == sequenced.seq);
    auto invalid_sequence = unpack(TracksV1Codec::encode(sequenced));
    invalid_sequence["seq"] = 1.0;
    assert(!TracksV1Codec::decode(pack(invalid_sequence)).valid);
    invalid_sequence["seq"] = true;
    assert(!TracksV1Codec::decode(pack(invalid_sequence)).valid);
    auto baseline = unpack(TracksV1Codec::encode(original));
    auto extended = baseline;
    extended["future_metadata"] = {{"display_hint", "example"},
                                   {"values", NativeJson::array({1, 2, 3})}};
    extended["tracks"][0]["display_color"] = "blue";
    const auto decoded = TracksV1Codec::decode(pack(extended));
    assert(decoded.valid && decoded.message.generation == original.generation);
    assert(decoded.message.fc_epoch == original.fc_epoch && decoded.message.tracks.size() == 1);
    assert(decoded.message.tracks[0].track_id == 1 && decoded.message.tracks[0].ground_anchor);
    auto no_epoch = extended;
    no_epoch.erase("fc_epoch");
    assert(TracksV1Codec::decode(pack(no_epoch)).valid);
    for (auto it = baseline.begin(); it != baseline.end(); ++it) {
        if (it.key() == "fc_epoch") {
            continue;
        }
        auto missing = extended;
        missing.erase(it.key());
        assert(!TracksV1Codec::decode(pack(missing)).valid);
    }
    for (auto it = baseline["tracks"][0].begin(); it != baseline["tracks"][0].end(); ++it) {
        if (it.key() == "ground" || it.key() == "ground_anchor") {
            continue;
        }
        auto missing = extended;
        missing["tracks"][0].erase(it.key());
        assert(!TracksV1Codec::decode(pack(missing)).valid);
    }
    assert(!TracksV1Codec::decode(duplicate_root(pack(extended), "frame_id")).valid);
    assert(!TracksV1Codec::decode(duplicate_root(pack(extended), "future_metadata")).valid);
    for (const auto& field : {"fc_epoch", "frame_id", "generation"}) {
        auto wrong = extended;
        wrong[field] = 1.0;
        assert(!TracksV1Codec::decode(pack(wrong)).valid);
    }
    auto bad_pose = extended;
    bad_pose["ground_pose"] = NativeJson::array({0, 0, 0});
    assert(!TracksV1Codec::decode(pack(bad_pose)).valid);
    auto bad_cov = extended;
    bad_cov["tracks"][0]["ground"][3] = 1.0;
    assert(!TracksV1Codec::decode(pack(bad_cov)).valid);
    auto no_anchor = extended;
    no_anchor["tracks"][0].erase("ground_anchor");
    assert(!TracksV1Codec::decode(pack(no_anchor)).valid);
    auto future_version = extended;
    future_version["schema"] = "argus.tracking.tracks.v6";
    assert(!TracksV1Codec::decode(pack(future_version)).valid);
    assert(!TracksV1Codec::decode(std::string(TracksV1Codec::kMaxMessageBytes + 1, 'x')).valid);
    for (const char* version :
         {kTracksV1Schema, kTracksV2Schema, kTracksV3Schema, kTracksV4Schema}) {
        auto old = original;
        old.schema = version;
        old.fc_epoch.reset();
        old.tracks[0].ground_anchor.reset();
        if (old.schema == kTracksV1Schema) {
            old.tracks[0].ground.reset();
        }
        if (old.schema != kTracksV4Schema) {
            old.ground_pose.reset();
        }
        if (old.schema == kTracksV1Schema || old.schema == kTracksV2Schema) {
            old.motion_sample_ns.reset();
        }
        auto legacy = unpack(TracksV1Codec::encode(old));
        assert(TracksV1Codec::decode(pack(legacy)).valid);
        auto unknown = legacy;
        unknown["new_metadata"] = 1;
        assert(!TracksV1Codec::decode(pack(unknown)).valid);
        legacy["tracks"][0]["display_color"] = "blue";
        assert(!TracksV1Codec::decode(pack(legacy)).valid);
    }
}
