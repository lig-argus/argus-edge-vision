#include "argus/perception_api_bus/dds_types.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "argus/perception_api/types.hpp"
#include "argus_msg/perception_observationsPubSubTypes.hpp"
#include "argus_msg/tracking_statusPubSubTypes.hpp"
#include "argus_msg/tracking_tracksPubSubTypes.hpp"
#include "argus_msg/vision_frame_gray8PubSubTypes.hpp"
#include "argus_msg/vision_statusPubSubTypes.hpp"

namespace argus::perception_api_bus {
namespace {

// 비어 있는 session_id 는 값이 없는 것으로 바꾼다
std::optional<std::string> session_of(const std::string& session_id) {
    if (session_id.empty()) {
        return std::nullopt;
    }
    return session_id;
}

// 탐지 결과의 박스는 원본 영상의 정수 픽셀이다 (detection.idl)
std::int32_t to_pixel(double value) { return static_cast<std::int32_t>(std::lround(value)); }

// vision/frame/gray8

bool frame_to_idl(const perception_api::Frame& frame, idl::VisionFrameGray8& sample) {
    sample.source(frame.source);
    sample.session_id(frame.session_id);
    sample.frame_id(frame.frame_id);
    sample.unix_ns(frame.unix_ns);
    sample.received_monotonic_ns(frame.received_monotonic_ns);
    sample.width(frame.width);
    sample.height(frame.height);
    sample.stride(frame.stride);
    sample.pixels(frame.pixels);
    return true;
}

bool frame_from_idl(const idl::VisionFrameGray8& sample, perception_api::Frame& frame) {
    if (sample.stride() < sample.width()) {
        return false;
    }
    frame.source = sample.source();
    frame.session_id = sample.session_id();
    frame.frame_id = sample.frame_id();
    frame.unix_ns = sample.unix_ns();
    frame.received_monotonic_ns = sample.received_monotonic_ns();
    frame.width = sample.width();
    frame.height = sample.height();
    frame.stride = sample.stride();
    if (sample.pixels().size() != frame.expected_pixel_bytes()) {
        return false;
    }
    frame.pixels = sample.pixels();
    return true;
}

// perception/observations

bool observations_to_idl(const perception_api::Observations& observations,
                         idl::PerceptionObservations& sample) {
    sample.source(observations.source);
    sample.session_id(observations.session_id.value_or(std::string{}));
    sample.frame_id(observations.frame_id);
    sample.image_width(observations.image_width);
    sample.image_height(observations.image_height);
    sample.model(observations.model);
    sample.received_monotonic_ns(observations.sample_monotonic_ns);
    sample.detections().clear();
    sample.detections().reserve(observations.detections.size());
    for (const perception_api::Detection& detection : observations.detections) {
        idl::Detection& added = sample.detections().emplace_back();
        added.class_id(detection.classification.class_id);
        added.label(detection.classification.label);
        added.confidence(detection.classification.confidence);
        added.x1(to_pixel(detection.box.x1));
        added.y1(to_pixel(detection.box.y1));
        added.x2(to_pixel(detection.box.x2));
        added.y2(to_pixel(detection.box.y2));
    }
    return true;
}

bool observations_from_idl(const idl::PerceptionObservations& sample,
                           perception_api::Observations& observations) {
    observations.source = sample.source();
    observations.session_id = session_of(sample.session_id());
    observations.frame_id = sample.frame_id();
    observations.sample_monotonic_ns = sample.received_monotonic_ns();
    observations.model = sample.model();
    observations.image_width = sample.image_width();
    observations.image_height = sample.image_height();
    observations.detections.clear();
    observations.detections.reserve(sample.detections().size());
    for (const idl::Detection& detection : sample.detections()) {
        perception_api::Detection& added = observations.detections.emplace_back();
        added.classification.class_id = detection.class_id();
        added.classification.label = detection.label();
        added.classification.confidence = detection.confidence();
        added.box.x1 = detection.x1();
        added.box.y1 = detection.y1();
        added.box.x2 = detection.x2();
        added.box.y2 = detection.y2();
    }
    return true;
}

// tracking/tracks

idl::TrackState to_idl(perception_api::TrackState state) {
    switch (state) {
        case perception_api::TrackState::Confirmed:
            return idl::TrackState::TRACK_STATE_CONFIRMED;
        case perception_api::TrackState::Predicted:
            return idl::TrackState::TRACK_STATE_PREDICTED;
    }
    return idl::TrackState::TRACK_STATE_UNSPECIFIED;
}

std::optional<perception_api::TrackState> from_idl(idl::TrackState state) {
    switch (state) {
        case idl::TrackState::TRACK_STATE_CONFIRMED:
            return perception_api::TrackState::Confirmed;
        case idl::TrackState::TRACK_STATE_PREDICTED:
            return perception_api::TrackState::Predicted;
        case idl::TrackState::TRACK_STATE_UNSPECIFIED:
            return std::nullopt;
    }
    // 이 코드에 정의되지 않은 값
    return std::nullopt;
}

void track_to_idl(const perception_api::Track& track, idl::TrackingTracks_Track& sample) {
    sample.track_id(track.track_id);
    sample.classification().class_id(track.classification.class_id);
    sample.classification().label(track.classification.label);
    sample.classification().confidence(static_cast<float>(track.classification.confidence));
    sample.box().x1(static_cast<float>(track.box.x1));
    sample.box().y1(static_cast<float>(track.box.y1));
    sample.box().x2(static_cast<float>(track.box.x2));
    sample.box().y2(static_cast<float>(track.box.y2));
    sample.state(to_idl(track.state));
    sample.real_detection(track.real_detection);
    sample.last_real_detection_monotonic_ns(track.last_real_detection_monotonic_ns);
    sample.ground().reset();
    if (track.ground.has_value()) {
        const perception_api::GroundEstimate& value = track.ground.value();
        idl::TrackingTracks_GroundEstimate ground;
        ground.north_m(value.north_m);
        ground.east_m(value.east_m);
        ground.var_north_m2(value.var_north_m2);
        ground.cov_north_east_m2(value.cov_north_east_m2);
        ground.var_east_m2(value.var_east_m2);
        ground.alignment_s(value.alignment_s);
        sample.ground(ground);
    }
    sample.ground_anchor().reset();
    if (track.ground_anchor.has_value()) {
        idl::TrackingTracks_GroundAnchor anchor;
        anchor.anchor_y_ratio(track.ground_anchor->anchor_y_ratio);
        anchor.height_m(track.ground_anchor->height_m);
        sample.ground_anchor(anchor);
    }
}

// 트랙 상태가 정의된 값이 아니면 nullopt
std::optional<perception_api::Track> track_from_idl(const idl::TrackingTracks_Track& sample) {
    const std::optional<perception_api::TrackState> state = from_idl(sample.state());
    if (!state.has_value()) {
        return std::nullopt;
    }
    perception_api::Track track;
    track.track_id = sample.track_id();
    track.classification.class_id = sample.classification().class_id();
    track.classification.label = sample.classification().label();
    track.classification.confidence = sample.classification().confidence();
    track.box.x1 = sample.box().x1();
    track.box.y1 = sample.box().y1();
    track.box.x2 = sample.box().x2();
    track.box.y2 = sample.box().y2();
    track.state = state.value();
    track.real_detection = sample.real_detection();
    track.last_real_detection_monotonic_ns = sample.last_real_detection_monotonic_ns();
    if (sample.ground().has_value()) {
        const idl::TrackingTracks_GroundEstimate& ground = *sample.ground();
        track.ground = perception_api::GroundEstimate{
            ground.north_m(),           ground.east_m(),      ground.var_north_m2(),
            ground.cov_north_east_m2(), ground.var_east_m2(), ground.alignment_s()};
    }
    if (sample.ground_anchor().has_value()) {
        track.ground_anchor = perception_api::GroundAnchor{sample.ground_anchor()->anchor_y_ratio(),
                                                           sample.ground_anchor()->height_m()};
    }
    return track;
}

bool tracks_to_idl(const perception_api::Tracks& tracks, idl::TrackingTracks& sample) {
    sample.source(tracks.source);
    sample.session_id(tracks.session_id.value_or(std::string{}));
    sample.frame_id(tracks.frame_id);
    sample.fc_epoch(tracks.fc_epoch);
    sample.generation(tracks.generation);
    sample.sample_monotonic_ns(tracks.sample_monotonic_ns);
    sample.image_width(tracks.image_width);
    sample.image_height(tracks.image_height);
    sample.ground_pose().reset();
    if (tracks.ground_pose.has_value()) {
        const perception_api::GroundPose& value = tracks.ground_pose.value();
        idl::TrackingTracks_GroundPose pose;
        pose.north_m(value.north_m);
        pose.east_m(value.east_m);
        pose.down_m(value.down_m);
        pose.roll_rad(value.roll_rad);
        pose.pitch_rad(value.pitch_rad);
        pose.yaw_rad(value.yaw_rad);
        sample.ground_pose(pose);
    }
    sample.tracks().clear();
    sample.tracks().reserve(tracks.tracks.size());
    for (const perception_api::Track& track : tracks.tracks) {
        track_to_idl(track, sample.tracks().emplace_back());
    }
    return true;
}

bool tracks_from_idl(const idl::TrackingTracks& sample, perception_api::Tracks& tracks) {
    tracks.source = sample.source();
    tracks.session_id = session_of(sample.session_id());
    tracks.frame_id = sample.frame_id();
    tracks.fc_epoch = sample.fc_epoch();
    tracks.generation = sample.generation();
    tracks.sample_monotonic_ns = sample.sample_monotonic_ns();
    tracks.image_width = sample.image_width();
    tracks.image_height = sample.image_height();
    tracks.ground_pose.reset();
    if (sample.ground_pose().has_value()) {
        const idl::TrackingTracks_GroundPose& pose = *sample.ground_pose();
        tracks.ground_pose =
            perception_api::GroundPose{pose.north_m(),  pose.east_m(),    pose.down_m(),
                                       pose.roll_rad(), pose.pitch_rad(), pose.yaw_rad()};
    }
    tracks.tracks.clear();
    tracks.tracks.reserve(sample.tracks().size());
    for (const idl::TrackingTracks_Track& received : sample.tracks()) {
        std::optional<perception_api::Track> track = track_from_idl(received);
        if (!track.has_value()) {
            return false;
        }
        tracks.tracks.push_back(std::move(track.value()));
    }
    return true;
}

bool tracking_status_to_idl(const perception_api::TrackingStatus& status,
                            idl::TrackingStatus& sample) {
    sample.source(status.source);
    sample.published_monotonic_ns(status.published_monotonic_ns);
    sample.generation(status.generation);
    sample.fc_fresh(status.fc_fresh);
    sample.cmc_valid(status.cmc_valid);
    sample.fps(status.fps);
    return true;
}

bool tracking_status_from_idl(const idl::TrackingStatus& sample,
                              perception_api::TrackingStatus& status) {
    status.source = sample.source();
    status.published_monotonic_ns = sample.published_monotonic_ns();
    status.generation = sample.generation();
    status.fc_fresh = sample.fc_fresh();
    status.cmc_valid = sample.cmc_valid();
    status.fps = sample.fps();
    return true;
}

bool vision_status_to_idl(const perception_api::VisionStatus& status, idl::VisionStatus& sample) {
    sample.source(status.source);
    sample.published_monotonic_ns(status.published_monotonic_ns);
    sample.camera_ok(status.camera_ok);
    sample.inference_ok(status.inference_ok);
    sample.infer_fps(status.infer_fps);
    sample.model_ver(status.model_ver);
    sample.image_width(status.image_width);
    sample.image_height(status.image_height);
    return true;
}

bool vision_status_from_idl(const idl::VisionStatus& sample, perception_api::VisionStatus& status) {
    status.source = sample.source();
    status.published_monotonic_ns = sample.published_monotonic_ns();
    status.camera_ok = sample.camera_ok();
    status.inference_ok = sample.inference_ok();
    status.infer_fps = sample.infer_fps();
    status.model_ver = sample.model_ver();
    status.image_width = sample.image_width();
    status.image_height = sample.image_height();
    return true;
}

}  // namespace

bus_dds::DdsTypeTable dds_types() {
    bus_dds::DdsTypeTable table;
    bus_dds::add_dds_type<perception_api::Frame, idl::VisionFrameGray8PubSubType>(
        table, &frame_to_idl, &frame_from_idl);
    bus_dds::add_dds_type<perception_api::Observations, idl::PerceptionObservationsPubSubType>(
        table, &observations_to_idl, &observations_from_idl);
    bus_dds::add_dds_type<perception_api::Tracks, idl::TrackingTracksPubSubType>(
        table, &tracks_to_idl, &tracks_from_idl);
    bus_dds::add_dds_type<perception_api::TrackingStatus, idl::TrackingStatusPubSubType>(
        table, &tracking_status_to_idl, &tracking_status_from_idl);
    bus_dds::add_dds_type<perception_api::VisionStatus, idl::VisionStatusPubSubType>(
        table, &vision_status_to_idl, &vision_status_from_idl);
    return table;
}

}  // namespace argus::perception_api_bus
