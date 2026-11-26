// native JSON/MessagePack 운용자 선택의 정수 정밀도와 클래스 신원을 보존한다.
#pragma once

#include <algorithm>
#include <limits>
#include <stdexcept>

#include "argus/common/identity/include/float_identity.hpp"
#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/target/include/target_selector.hpp"

namespace follow {
inline NativeJson resolve_icd_target(const NativeJson& request, const TracksV1& frame) {
    auto resolved = request;
    if (request.value("action", "") != "track_start" || request.contains("label")) {
        return resolved;
    }
    if (request.at("sensor") != frame.source || request.at("generation") != frame.generation) {
        throw std::invalid_argument("ICD target source/generation mismatch");
    }
    const TrackRecordV1* candidate = nullptr;
    for (const auto& track : frame.tracks) {
        if (request.at("track_id") == track.track_id && request.at("class_id") == track.class_id) {
            if (candidate || track.state != "CONFIRMED" || !track.real_detection) {
                throw std::invalid_argument("ICD target is ambiguous or not confirmed");
            }
            candidate = &track;
        }
    }
    if (!candidate) {
        throw std::invalid_argument("ICD target not in current tracks");
    }
    resolved["label"] = candidate->label;
    return resolved;
}

inline TargetCommand native_target_request(const NativeJson& request, const std::string& uid) {
    const auto action = request.at("action").get<std::string>();
    TargetCommand command{request.at("request_id").get<std::string>(),
                          action == "start"         ? TargetCommandAction::Start
                          : action == "track_start" ? TargetCommandAction::SelectAndStart
                                                    : TargetCommandAction::Select,
                          0, 0, uid};
    if (action != "select" && action != "start" && action != "track_start") {
        throw std::invalid_argument("invalid native target action");
    }
    if (request.contains("class_id") != request.contains("label") ||
        (action == "start" && request.contains("class_id")) ||
        (action == "track_start" &&
         (!request.contains("class_id") || !request.contains("sensor")))) {
        throw std::invalid_argument("class_id and label require a select pair");
    }
    if (action == "start") {
        return command;
    }
    const auto& generation = request.at("generation");
    if (!generation.is_number_integer() ||
        (!generation.is_number_unsigned() && generation.get<std::int64_t>() < 0)) {
        throw std::invalid_argument("generation must be a uint64 integer");
    }
    command.generation = generation.get<std::uint64_t>();
    const auto& id = request.at("track_id");
    if (!id.is_number_integer() ||
        (id.is_number_unsigned() &&
         id.get<std::uint64_t>() > std::uint64_t(std::numeric_limits<std::int64_t>::max())) ||
        id.get<std::int64_t>() <= 0) {
        throw std::invalid_argument("track_id must be a positive int64 integer");
    }
    command.track_id = id.get<std::int64_t>();
    if (!valid_float_identity(command.generation) ||
        !valid_float_identity(static_cast<std::uint64_t>(command.track_id))) {
        throw std::invalid_argument("generation and track_id must be integers in 1..16777215");
    }
    if (request.contains("sensor")) {
        auto sensor = request.at("sensor").get<std::string>();
        if (sensor.empty() || sensor.size() > 128 ||
            std::any_of(sensor.begin(), sensor.end(),
                        [](unsigned char c) { return c < 32 || c == 127; })) {
            throw std::invalid_argument("invalid candidate sensor");
        }
        command.sensor = std::move(sensor);
    }
    if (request.contains("class_id")) {
        const auto& class_id = request.at("class_id");
        if (!class_id.is_number_integer() ||
            (class_id.is_number_unsigned() &&
             class_id.get<std::uint64_t>() > std::uint64_t(kFloatIdentityMax)) ||
            class_id.get<std::int64_t>() < 0 || class_id.get<std::int64_t>() > kFloatIdentityMax) {
            throw std::invalid_argument("command class_id must be an integer in 0..16777215");
        }
        auto label = request.at("label").get<std::string>();
        if (label.empty() || label.size() > 128 ||
            std::any_of(label.begin(), label.end(),
                        [](unsigned char c) { return c < 32 || c == 127; })) {
            throw std::invalid_argument("invalid candidate label");
        }
        command.class_id = class_id.get<std::int32_t>();
        command.label = std::move(label);
    }
    return command;
}
}  // namespace follow
