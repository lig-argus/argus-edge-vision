#include "argus/perception_api_bus/zmq_codecs.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "argus/perception_api/types.hpp"
#include "argus_msg/perception_observations.pb.h"
#include "argus_msg/tracking_status.pb.h"
#include "argus_msg/tracking_tracks.pb.h"
#include "argus_msg/vision_frame_gray8.pb.h"
#include "argus_msg/vision_status.pb.h"

namespace argus::perception_api_bus {
namespace {

using bus_zmq::Bytes;

// 직렬화에 실패하면 빈 바이트를 반환한다. 실패하는 경우는 메시지 크기가 2 GB 를 넘을 때뿐이다
template <typename Message>
Bytes to_bytes(const Message& message) {
    Bytes body(message.ByteSizeLong());
    if (!message.SerializeToArray(body.data(), static_cast<std::int32_t>(body.size()))) {
        body.clear();
    }
    return body;
}

template <typename Message>
bool parse(const Bytes& body, Message& message) {
    return message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size()));
}

// 비어 있는 session_id 는 값이 없는 것으로 바꾼다
std::optional<std::string> session_of(const std::string& session_id) {
    if (session_id.empty()) {
        return std::nullopt;
    }
    return session_id;
}

// 탐지 결과의 박스는 원본 영상의 정수 픽셀이다 (detection.proto)
std::int32_t to_pixel(double value) { return static_cast<std::int32_t>(std::lround(value)); }

// vision/frame/gray8

bool encode_frame(const perception_api::Frame& frame, std::uint64_t /*sequence*/, Bytes& body,
                  Bytes& binary) {
    msg::VisionFrameGray8 message;
    message.set_source(frame.source);
    message.set_session_id(frame.session_id);
    message.set_frame_id(frame.frame_id);
    message.set_seq(frame.frame_id);  // seq 는 frame_id 와 같은 값 (vision_frame_gray8.proto)
    message.set_unix_ns(frame.unix_ns);
    message.set_received_monotonic_ns(frame.received_monotonic_ns);
    message.set_width(frame.width);
    message.set_height(frame.height);
    message.set_stride(frame.stride);
    body = to_bytes(message);
    binary = frame.pixels;
    return true;
}

bool decode_frame(const Bytes& body, const Bytes& binary, perception_api::Frame& frame,
                  std::uint64_t& sequence) {
    msg::VisionFrameGray8 message;
    if (!parse(body, message) || message.stride() < message.width()) {
        return false;
    }
    frame.source = message.source();
    frame.session_id = message.session_id();
    frame.frame_id = message.frame_id();
    frame.unix_ns = message.unix_ns();
    frame.received_monotonic_ns = message.received_monotonic_ns();
    frame.width = message.width();
    frame.height = message.height();
    frame.stride = message.stride();
    if (binary.size() != frame.expected_pixel_bytes()) {
        return false;
    }
    frame.pixels = binary;
    sequence = message.seq();
    return true;
}

// perception/observations

Bytes encode_observations(const perception_api::Observations& observations, std::uint64_t seq) {
    msg::PerceptionObservations message;
    message.set_source(observations.source);
    message.set_session_id(observations.session_id.value_or(std::string{}));
    message.set_frame_id(observations.frame_id);
    message.set_seq(seq);
    message.set_image_width(observations.image_width);
    message.set_image_height(observations.image_height);
    message.set_model(observations.model);
    message.set_received_monotonic_ns(observations.sample_monotonic_ns);
    for (const perception_api::Detection& detection : observations.detections) {
        msg::Detection* const added = message.add_detections();
        added->set_class_id(detection.classification.class_id);
        added->set_label(detection.classification.label);
        added->set_confidence(detection.classification.confidence);
        added->set_x1(to_pixel(detection.box.x1));
        added->set_y1(to_pixel(detection.box.y1));
        added->set_x2(to_pixel(detection.box.x2));
        added->set_y2(to_pixel(detection.box.y2));
    }
    return to_bytes(message);
}

std::optional<perception_api::Observations> decode_observations(const Bytes& body,
                                                                std::uint64_t& seq) {
    msg::PerceptionObservations message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    perception_api::Observations observations;
    observations.source = message.source();
    observations.session_id = session_of(message.session_id());
    observations.frame_id = message.frame_id();
    observations.sample_monotonic_ns = message.received_monotonic_ns();
    observations.model = message.model();
    observations.image_width = message.image_width();
    observations.image_height = message.image_height();
    observations.detections.reserve(static_cast<std::size_t>(message.detections_size()));
    for (const msg::Detection& detection : message.detections()) {
        perception_api::Detection& added = observations.detections.emplace_back();
        added.classification.class_id = detection.class_id();
        added.classification.label = detection.label();
        added.classification.confidence = detection.confidence();
        added.box.x1 = detection.x1();
        added.box.y1 = detection.y1();
        added.box.x2 = detection.x2();
        added.box.y2 = detection.y2();
    }
    seq = message.seq();
    return observations;
}

// tracking/tracks

msg::TrackState to_message(perception_api::TrackState state) {
    return state == perception_api::TrackState::Predicted ? msg::TRACK_STATE_PREDICTED
                                                          : msg::TRACK_STATE_CONFIRMED;
}

std::optional<perception_api::TrackState> from_message(msg::TrackState state) {
    switch (state) {
        case msg::TRACK_STATE_CONFIRMED:
            return perception_api::TrackState::Confirmed;
        case msg::TRACK_STATE_PREDICTED:
            return perception_api::TrackState::Predicted;
        default:
            return std::nullopt;
    }
}

void to_message(const perception_api::Track& track, msg::TrackingTracks::Track& message) {
    message.set_track_id(track.track_id);
    msg::TrackingTracks::Classification* const classification = message.mutable_classification();
    classification->set_class_id(track.classification.class_id);
    classification->set_label(track.classification.label);
    classification->set_confidence(static_cast<float>(track.classification.confidence));
    msg::TrackingTracks::PixelBox* const box = message.mutable_box();
    box->set_x1(static_cast<float>(track.box.x1));
    box->set_y1(static_cast<float>(track.box.y1));
    box->set_x2(static_cast<float>(track.box.x2));
    box->set_y2(static_cast<float>(track.box.y2));
    message.set_state(to_message(track.state));
    message.set_real_detection(track.real_detection);
    message.set_last_real_detection_monotonic_ns(track.last_real_detection_monotonic_ns);
    if (track.ground.has_value()) {
        const perception_api::GroundEstimate& value = track.ground.value();
        msg::TrackingTracks::GroundEstimate* const ground = message.mutable_ground();
        ground->set_north_m(value.north_m);
        ground->set_east_m(value.east_m);
        ground->set_var_north_m2(value.var_north_m2);
        ground->set_cov_north_east_m2(value.cov_north_east_m2);
        ground->set_var_east_m2(value.var_east_m2);
        ground->set_alignment_s(value.alignment_s);
    }
    if (track.ground_anchor.has_value()) {
        const perception_api::GroundAnchor& value = track.ground_anchor.value();
        msg::TrackingTracks::GroundAnchor* const anchor = message.mutable_ground_anchor();
        anchor->set_anchor_y_ratio(value.anchor_y_ratio);
        anchor->set_height_m(value.height_m);
    }
}

// 트랙 상태가 정의된 값이 아니면 nullopt
std::optional<perception_api::Track> from_message(const msg::TrackingTracks::Track& message) {
    const std::optional<perception_api::TrackState> state = from_message(message.state());
    if (!state.has_value()) {
        return std::nullopt;
    }
    perception_api::Track track;
    track.track_id = message.track_id();
    track.classification.class_id = message.classification().class_id();
    track.classification.label = message.classification().label();
    track.classification.confidence = message.classification().confidence();
    track.box.x1 = message.box().x1();
    track.box.y1 = message.box().y1();
    track.box.x2 = message.box().x2();
    track.box.y2 = message.box().y2();
    track.state = state.value();
    track.real_detection = message.real_detection();
    track.last_real_detection_monotonic_ns = message.last_real_detection_monotonic_ns();
    if (message.has_ground()) {
        const msg::TrackingTracks::GroundEstimate& ground = message.ground();
        track.ground = perception_api::GroundEstimate{
            ground.north_m(),           ground.east_m(),      ground.var_north_m2(),
            ground.cov_north_east_m2(), ground.var_east_m2(), ground.alignment_s()};
    }
    if (message.has_ground_anchor()) {
        track.ground_anchor = perception_api::GroundAnchor{message.ground_anchor().anchor_y_ratio(),
                                                           message.ground_anchor().height_m()};
    }
    return track;
}

Bytes encode_tracks(const perception_api::Tracks& tracks, std::uint64_t seq) {
    msg::TrackingTracks message;
    message.set_source(tracks.source);
    message.set_session_id(tracks.session_id.value_or(std::string{}));
    message.set_frame_id(tracks.frame_id);
    message.set_seq(seq);
    message.set_fc_epoch(tracks.fc_epoch);
    message.set_generation(tracks.generation);
    message.set_sample_monotonic_ns(tracks.sample_monotonic_ns);
    message.set_image_width(tracks.image_width);
    message.set_image_height(tracks.image_height);
    if (tracks.ground_pose.has_value()) {
        const perception_api::GroundPose& value = tracks.ground_pose.value();
        msg::TrackingTracks::GroundPose* const pose = message.mutable_ground_pose();
        pose->set_north_m(value.north_m);
        pose->set_east_m(value.east_m);
        pose->set_down_m(value.down_m);
        pose->set_roll_rad(value.roll_rad);
        pose->set_pitch_rad(value.pitch_rad);
        pose->set_yaw_rad(value.yaw_rad);
    }
    for (const perception_api::Track& track : tracks.tracks) {
        to_message(track, *message.add_tracks());
    }
    return to_bytes(message);
}

std::optional<perception_api::Tracks> decode_tracks(const Bytes& body, std::uint64_t& seq) {
    msg::TrackingTracks message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    perception_api::Tracks tracks;
    tracks.source = message.source();
    tracks.session_id = session_of(message.session_id());
    tracks.frame_id = message.frame_id();
    tracks.fc_epoch = message.fc_epoch();
    tracks.generation = message.generation();
    tracks.sample_monotonic_ns = message.sample_monotonic_ns();
    tracks.image_width = message.image_width();
    tracks.image_height = message.image_height();
    if (message.has_ground_pose()) {
        const msg::TrackingTracks::GroundPose& pose = message.ground_pose();
        tracks.ground_pose =
            perception_api::GroundPose{pose.north_m(),  pose.east_m(),    pose.down_m(),
                                       pose.roll_rad(), pose.pitch_rad(), pose.yaw_rad()};
    }
    tracks.tracks.reserve(static_cast<std::size_t>(message.tracks_size()));
    for (const msg::TrackingTracks::Track& track : message.tracks()) {
        std::optional<perception_api::Track> decoded = from_message(track);
        if (!decoded.has_value()) {
            return std::nullopt;
        }
        tracks.tracks.push_back(std::move(decoded.value()));
    }
    seq = message.seq();
    return tracks;
}

Bytes encode_tracking_status(const perception_api::TrackingStatus& status, std::uint64_t seq) {
    msg::TrackingStatus message;
    message.set_source(status.source);
    message.set_seq(seq);
    message.set_published_monotonic_ns(status.published_monotonic_ns);
    message.set_generation(status.generation);
    message.set_fc_fresh(status.fc_fresh);
    message.set_cmc_valid(status.cmc_valid);
    message.set_fps(status.fps);
    return to_bytes(message);
}

std::optional<perception_api::TrackingStatus> decode_tracking_status(const Bytes& body,
                                                                     std::uint64_t& seq) {
    msg::TrackingStatus message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    seq = message.seq();
    perception_api::TrackingStatus status;
    status.source = message.source();
    status.published_monotonic_ns = message.published_monotonic_ns();
    status.generation = message.generation();
    status.fc_fresh = message.fc_fresh();
    status.cmc_valid = message.cmc_valid();
    status.fps = message.fps();
    return status;
}

Bytes encode_vision_status(const perception_api::VisionStatus& status, std::uint64_t seq) {
    msg::VisionStatus message;
    message.set_source(status.source);
    message.set_seq(seq);
    message.set_published_monotonic_ns(status.published_monotonic_ns);
    message.set_camera_ok(status.camera_ok);
    message.set_inference_ok(status.inference_ok);
    message.set_infer_fps(status.infer_fps);
    message.set_model_ver(status.model_ver);
    message.set_image_width(status.image_width);
    message.set_image_height(status.image_height);
    return to_bytes(message);
}

std::optional<perception_api::VisionStatus> decode_vision_status(const Bytes& body,
                                                                 std::uint64_t& seq) {
    msg::VisionStatus message;
    if (!parse(body, message)) {
        return std::nullopt;
    }
    seq = message.seq();
    perception_api::VisionStatus status;
    status.source = message.source();
    status.published_monotonic_ns = message.published_monotonic_ns();
    status.camera_ok = message.camera_ok();
    status.inference_ok = message.inference_ok();
    status.infer_fps = message.infer_fps();
    status.model_ver = message.model_ver();
    status.image_width = message.image_width();
    status.image_height = message.image_height();
    return status;
}

// 본문만 있는 토픽 메시지. 발행 번호를 본문의 seq 에 싣는다. 바이너리가 있는 메시지는 형식이 맞지
// 않는 것이다
template <typename T>
void add_body_only_codec(bus_zmq::ZmqCodecTable& table, Bytes (*encode)(const T&, std::uint64_t),
                         std::optional<T> (*decode)(const Bytes&, std::uint64_t&)) {
    bus_zmq::add_zmq_codec<T>(
        table,
        [encode](const T& value, std::uint64_t sequence, Bytes& body, Bytes& /*binary*/) {
            body = encode(value, sequence);
            return true;
        },
        [decode](const Bytes& body, const Bytes& binary, T& value, std::uint64_t& sequence) {
            if (!binary.empty()) {
                return false;
            }
            std::optional<T> decoded = decode(body, sequence);
            if (!decoded.has_value()) {
                return false;
            }
            value = std::move(decoded.value());
            return true;
        });
}

}  // namespace

bus_zmq::ZmqCodecTable zmq_codecs() {
    bus_zmq::ZmqCodecTable table;
    bus_zmq::add_zmq_codec<perception_api::Frame>(table, encode_frame, decode_frame);
    add_body_only_codec<perception_api::Observations>(table, encode_observations,
                                                      decode_observations);
    add_body_only_codec<perception_api::Tracks>(table, encode_tracks, decode_tracks);
    add_body_only_codec<perception_api::TrackingStatus>(table, encode_tracking_status,
                                                        decode_tracking_status);
    add_body_only_codec<perception_api::VisionStatus>(table, encode_vision_status,
                                                      decode_vision_status);
    return table;
}

}  // namespace argus::perception_api_bus
