// v5의 선택적 확장은 무시하되 필수 필드·중복·자료형·범위 검사를 유지한다.
#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"

#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "argus/common/serialization/include/message_pack.hpp"

namespace follow {
namespace {
using Object = msgpack::object;

// 2026-11-19 선택 확장용: 숫자 자료형인지만 본다 (비유한 값 판단은 사용처가 한다).
bool numeric_object(const Object& value) {
    return value.type == msgpack::type::FLOAT32 || value.type == msgpack::type::FLOAT64 ||
           value.type == msgpack::type::POSITIVE_INTEGER ||
           value.type == msgpack::type::NEGATIVE_INTEGER;
}

void validate_track(const TrackRecordV1& track) {
    if (!valid_track_record(track)) {
        throw std::runtime_error("invalid track record");
    }
}

TrackRecordV1 read_track(const Object& object, bool additive_v5) {
    if (object.type != msgpack::type::MAP || object.via.map.size < 11 ||
        (!additive_v5 && object.via.map.size > 13)) {
        throw std::runtime_error("invalid track record field count");
    }
    TrackRecordV1 track;
    std::unordered_set<std::string> track_seen;
    for (std::uint32_t j = 0; j < object.via.map.size; ++j) {
        const auto& pair = object.via.map.ptr[j];
        const auto key = msgpack_read_string(pair.key, 64);
        if (!track_seen.insert(key).second) {
            throw std::runtime_error("duplicate track field");
        }
        const auto& value = pair.val;
        if (key == "track_id") {
            track.track_id = msgpack_read_i64(value);
        } else if (key == "class_id") {
            const auto n = msgpack_read_i64(value);
            if (n < 0 || n > std::numeric_limits<std::int32_t>::max()) {
                throw std::runtime_error("class ID out of range");
            }
            track.class_id = static_cast<std::int32_t>(n);
        } else if (key == "label") {
            track.label = msgpack_read_string(value, 128);
        } else if (key == "confidence") {
            track.confidence = msgpack_read_finite_number(value);
        } else if (key == "x1") {
            track.x1 = msgpack_read_finite_number(value);
        } else if (key == "y1") {
            track.y1 = msgpack_read_finite_number(value);
        } else if (key == "x2") {
            track.x2 = msgpack_read_finite_number(value);
        } else if (key == "y2") {
            track.y2 = msgpack_read_finite_number(value);
        } else if (key == "state") {
            track.state = msgpack_read_string(value, 16);
        } else if (key == "real_detection") {
            track.real_detection = msgpack_read_bool(value);
        } else if (key == "last_real_detection_monotonic_ns") {
            track.last_real_detection_monotonic_ns = msgpack_read_i64(value);
        } else if (key == "ground") {
            if (value.type != msgpack::type::ARRAY || value.via.array.size != 6) {
                throw std::runtime_error("ground must contain six numbers");
            }
            std::array<double, 6> ground;
            for (unsigned k = 0; k < 6; ++k) {
                ground[k] = msgpack_read_finite_number(value.via.array.ptr[k]);
            }
            track.ground = ground;
        } else if (key == "ground_anchor") {
            if (value.type != msgpack::type::ARRAY || value.via.array.size != 2) {
                throw std::runtime_error("ground anchor must contain two numbers");
            }
            track.ground_anchor =
                std::array<double, 2>{msgpack_read_finite_number(value.via.array.ptr[0]),
                                      msgpack_read_finite_number(value.via.array.ptr[1])};
        } else if (key == "thermal") {
            if (value.type != msgpack::type::ARRAY ||
                value.via.array.size != kThermalSignatureSize) {
                throw std::runtime_error("thermal must contain the signature numbers");
            }
            ThermalSignature thermal{};
            for (unsigned k = 0; k < kThermalSignatureSize; ++k) {
                thermal.at(k) = msgpack_read_finite_number(value.via.array.ptr[k]);
            }
            if (!valid_thermal_signature(thermal)) {
                throw std::runtime_error("invalid thermal signature");
            }
            track.thermal = thermal;
        } else if (!additive_v5) {
            throw std::runtime_error("unknown track field");
        }
    }
    if (additive_v5) {
        for (const auto* key :
             {"track_id", "class_id", "label", "confidence", "x1", "y1", "x2", "y2", "state",
              "real_detection", "last_real_detection_monotonic_ns"}) {
            if (!track_seen.count(key)) {
                throw std::runtime_error("missing required v5 track field");
            }
        }
    }
    validate_track(track);
    return track;
}
}  // namespace

std::string TracksV1Codec::encode(const TracksV1& message) {
    if (!valid_tracks_envelope(message) || message.tracks.size() > 512) {
        throw std::invalid_argument("invalid tracks envelope");
    }
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> packer(buffer);
    packer.pack_map((posed_tracks_schema(message.schema) ? 15
                     : message.schema == kTracksV3Schema ? 14
                                                         : 13) +
                    (message.fc_epoch ? 1 : 0) + (message.seq ? 1 : 0) +
                    (message.ground_plane_down_m ? 1 : 0) +
                    (message.projection_attitude_deg ? 1 : 0));
    msgpack_pack_field(packer, "schema", message.schema);
    msgpack_pack_field(packer, "source", message.source);
    msgpack_pack_field(packer, "frame_id", message.frame_id);
    if (message.seq) {
        msgpack_pack_field(packer, "seq", *message.seq);
    }
    msgpack_pack_field(packer, "sample_monotonic_ns", message.sample_monotonic_ns);
    msgpack_pack_field(packer, "clock_domain", message.clock_domain);
    msgpack_pack_field(packer, "timestamp_source", message.timestamp_source);
    msgpack_pack_optional(packer, "exposure_monotonic_ns", message.exposure_monotonic_ns);
    msgpack_pack_field(packer, "exposure_time_valid", message.exposure_time_valid);
    msgpack_pack_field(packer, "image_width", message.image_width);
    msgpack_pack_field(packer, "image_height", message.image_height);
    msgpack_pack_field(packer, "coordinate_frame", message.coordinate_frame);
    msgpack_pack_field(packer, "generation", message.generation);
    if (message.fc_epoch) {
        msgpack_pack_field(packer, "fc_epoch", *message.fc_epoch);
    }
    if (message.motion_sample_ns) {
        msgpack_pack_field(packer, "motion_sample_ns", *message.motion_sample_ns);
    }
    if (posed_tracks_schema(message.schema)) {
        packer.pack(std::string("ground_pose"));
        if (!message.ground_pose) {
            packer.pack_nil();
        } else {
            packer.pack_array(6);
            for (double v : *message.ground_pose) {
                packer.pack(v);
            }
        }
    }
    // 2026-11-19 v5 선택 확장 (valid_tracks_envelope가 v5만 허용한다).
    if (message.ground_plane_down_m) {
        msgpack_pack_field(packer, "ground_plane_down_m", *message.ground_plane_down_m);
    }
    if (message.projection_attitude_deg) {
        packer.pack(std::string("projection_attitude_deg"));
        packer.pack_array(3);
        for (double v : *message.projection_attitude_deg) {
            packer.pack(v);
        }
    }
    packer.pack(std::string("tracks"));
    packer.pack_array(static_cast<std::uint32_t>(message.tracks.size()));
    for (const auto& track : message.tracks) {
        validate_track(track);
        if (!track_fits_frame(track, message) || track.last_real_detection_monotonic_ns < 0) {
            throw std::invalid_argument("track box or time exceeds envelope");
        }
        if (track.ground && !ground_tracks_schema(message.schema)) {
            throw std::invalid_argument("ground observations require tracks v2");
        }
        if ((track.ground_anchor.has_value() && message.schema != kTracksV5Schema) ||
            (message.schema == kTracksV5Schema && track.ground && !track.ground_anchor)) {
            throw std::invalid_argument("ground anchor requires v5 and explicit observation");
        }
        if (track.thermal.has_value() &&
            (message.schema != static_cast<const char*>(kTracksV5Schema) ||
             !valid_thermal_signature(track.thermal.value_or(ThermalSignature{})))) {
            throw std::invalid_argument("thermal signature requires v5 and valid values");
        }
        std::uint32_t track_fields = 11;
        if (track.ground) {
            track_fields = 12;
        }
        if (track.ground_anchor) {
            track_fields = 13;
        }
        if (track.thermal) {
            ++track_fields;
        }
        packer.pack_map(track_fields);
        msgpack_pack_field(packer, "track_id", track.track_id);
        msgpack_pack_field(packer, "class_id", track.class_id);
        msgpack_pack_field(packer, "label", track.label);
        msgpack_pack_field(packer, "confidence", track.confidence);
        msgpack_pack_field(packer, "x1", track.x1);
        msgpack_pack_field(packer, "y1", track.y1);
        msgpack_pack_field(packer, "x2", track.x2);
        msgpack_pack_field(packer, "y2", track.y2);
        msgpack_pack_field(packer, "state", track.state);
        msgpack_pack_field(packer, "real_detection", track.real_detection);
        msgpack_pack_field(packer, "last_real_detection_monotonic_ns",
                           track.last_real_detection_monotonic_ns);
        if (track.ground) {
            packer.pack(std::string("ground"));
            packer.pack_array(6);
            for (double value : *track.ground) {
                packer.pack(value);
            }
        }
        if (track.ground_anchor) {
            packer.pack(std::string("ground_anchor"));
            packer.pack_array(2);
            for (double v : *track.ground_anchor) {
                packer.pack(v);
            }
        }
        if (track.thermal.has_value()) {
            packer.pack(std::string("thermal"));
            packer.pack_array(static_cast<std::uint32_t>(kThermalSignatureSize));
            for (const double v : track.thermal.value_or(ThermalSignature{})) {
                packer.pack(v);
            }
        }
    }
    if (buffer.size() > kMaxMessageBytes) {
        throw std::length_error("tracks message too large");
    }
    return {buffer.data(), buffer.size()};
}

TracksDecodeResult TracksV1Codec::decode(std::string_view packed) {
    TracksDecodeResult result;
    try {
        if (packed.empty() || packed.size() > kMaxMessageBytes) {
            throw std::runtime_error("invalid tracks message size");
        }
        std::size_t offset = 0;
        const auto handle = msgpack::unpack(packed.data(), packed.size(), offset);
        if (offset != packed.size()) {
            throw std::runtime_error("trailing MessagePack bytes");
        }
        const auto& root = handle.get();
        if (root.type != msgpack::type::MAP || root.via.map.size < 13) {
            throw std::runtime_error("invalid tracks envelope field count");
        }
        // Schema may appear anywhere in the map. Only v5 admits optional additions.
        bool additive_v5 = false;
        for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
            const auto& pair = root.via.map.ptr[i];
            if (msgpack_read_string(pair.key, 64) == "schema") {
                additive_v5 = msgpack_read_string(pair.val, 96) == kTracksV5Schema;
            }
        }
        if (!additive_v5 && root.via.map.size > 16) {
            throw std::runtime_error("too many legacy tracks fields");
        }
        // v5 explicitly checks every required key, so an extension cannot stand
        // in for a missing safety field. Duplicate unknown keys are also rejected.
        std::unordered_set<std::string> seen;
        const Object* track_array = nullptr;
        for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
            const auto& pair = root.via.map.ptr[i];
            const auto key = msgpack_read_string(pair.key, 64);
            if (!seen.insert(key).second) {
                throw std::runtime_error("duplicate tracks field");
            }
            const auto& value = pair.val;
            if (key == "schema") {
                result.message.schema = msgpack_read_string(value, 96);
            } else if (key == "source") {
                result.message.source = msgpack_read_string(value, 128);
            } else if (key == "frame_id") {
                result.message.frame_id = msgpack_read_u64(value);
            } else if (key == "seq" && additive_v5) {
                result.message.seq = msgpack_read_u64(value);
            } else if (key == "sample_monotonic_ns") {
                result.message.sample_monotonic_ns = msgpack_read_i64(value);
            } else if (key == "clock_domain") {
                result.message.clock_domain = msgpack_read_string(value, 64);
            } else if (key == "timestamp_source") {
                result.message.timestamp_source = msgpack_read_string(value, 64);
            } else if (key == "exposure_monotonic_ns") {
                result.message.exposure_monotonic_ns = msgpack_read_optional_i64(value);
            } else if (key == "exposure_time_valid") {
                result.message.exposure_time_valid = msgpack_read_bool(value);
            } else if (key == "image_width") {
                const auto n = msgpack_read_u64(value);
                if (n > std::numeric_limits<std::uint32_t>::max()) {
                    throw std::runtime_error("image width overflow");
                }
                result.message.image_width = static_cast<std::uint32_t>(n);
            } else if (key == "image_height") {
                const auto n = msgpack_read_u64(value);
                if (n > std::numeric_limits<std::uint32_t>::max()) {
                    throw std::runtime_error("image height overflow");
                }
                result.message.image_height = static_cast<std::uint32_t>(n);
            } else if (key == "coordinate_frame") {
                result.message.coordinate_frame = msgpack_read_string(value, 64);
            } else if (key == "generation") {
                result.message.generation = msgpack_read_u64(value);
            } else if (key == "fc_epoch") {
                result.message.fc_epoch = msgpack_read_u64(value);
            } else if (key == "motion_sample_ns") {
                result.message.motion_sample_ns = msgpack_read_i64(value);
            } else if (key == "ground_pose") {
                if (value.type != msgpack::type::NIL) {
                    if (value.type != msgpack::type::ARRAY || value.via.array.size != 6) {
                        throw std::runtime_error("ground pose must contain six numbers");
                    }
                    std::array<double, 6> pose;
                    for (unsigned k = 0; k < 6; ++k) {
                        pose[k] = msgpack_read_finite_number(value.via.array.ptr[k]);
                    }
                    result.message.ground_pose = pose;
                }
            } else if (key == "ground_plane_down_m" && additive_v5) {
                // 2026-11-19 선택 확장: 자료형이 다르면 없는 것으로 본다 (구버전처럼 프레임은
                // 받는다).
                if (numeric_object(value)) {
                    result.message.ground_plane_down_m = value.as<double>();
                }
            } else if (key == "projection_attitude_deg" && additive_v5) {
                if (value.type == msgpack::type::ARRAY && value.via.array.size == 3 &&
                    numeric_object(value.via.array.ptr[0]) &&
                    numeric_object(value.via.array.ptr[1]) &&
                    numeric_object(value.via.array.ptr[2])) {
                    result.message.projection_attitude_deg = std::array<double, 3>{
                        value.via.array.ptr[0].as<double>(), value.via.array.ptr[1].as<double>(),
                        value.via.array.ptr[2].as<double>()};
                }
            } else if (key == "tracks") {
                track_array = &value;
            } else if (!additive_v5) {
                throw std::runtime_error("unknown tracks field");
            }
        }
        if (additive_v5) {
            for (const auto* key :
                 {"schema", "source", "frame_id", "sample_monotonic_ns", "clock_domain",
                  "timestamp_source", "exposure_monotonic_ns", "exposure_time_valid", "image_width",
                  "image_height", "coordinate_frame", "generation", "tracks", "motion_sample_ns",
                  "ground_pose"}) {
                if (!seen.count(key)) {
                    throw std::runtime_error("missing required v5 envelope field");
                }
            }
        }
        if (!valid_tracks_envelope(result.message) ||
            (!additive_v5 &&
             root.via.map.size != ((posed_tracks_schema(result.message.schema) ? 15u
                                    : result.message.schema == kTracksV3Schema ? 14u
                                                                               : 13u) +
                                   (result.message.fc_epoch ? 1u : 0u))) ||
            !track_array || track_array->type != msgpack::type::ARRAY ||
            track_array->via.array.size > 512) {
            throw std::runtime_error("invalid tracks envelope values");
        }
        for (std::uint32_t i = 0; i < track_array->via.array.size; ++i) {
            auto track = read_track(track_array->via.array.ptr[i], additive_v5);
            if (track.thermal &&
                result.message.schema != static_cast<const char*>(kTracksV5Schema)) {
                throw std::runtime_error("thermal signature requires v5");
            }
            if ((track.ground_anchor && result.message.schema != kTracksV5Schema) ||
                (result.message.schema == kTracksV5Schema && track.ground &&
                 !track.ground_anchor)) {
                throw std::runtime_error("ground anchor/version mismatch");
            }
            if (track.ground && !ground_tracks_schema(result.message.schema)) {
                throw std::runtime_error("ground observations require tracks v2");
            }
            if (!track_fits_frame(track, result.message)) {
                throw std::runtime_error("track box/time exceeds envelope");
            }
            result.message.tracks.push_back(std::move(track));
        }
        result.valid = true;
    } catch (const std::exception& error) {
        result.error = error.what();
    }
    return result;
}
}  // namespace follow
