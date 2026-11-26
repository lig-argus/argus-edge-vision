#pragma once

// 2026-11-23 사용자 지시 "c++쪽 고쳐줘": 영상 준비 확인의 프레임 판정.
// src/argus/runtime/streaming/preview_producer_supervisor.py ensure()의 overlay_ready,
// gray8_ready와 같은 규칙이다. 2026-11-15 C++ 탐지 프로세스 전환(ARGUS_WORKER_NO_JPEG=1)으로 JPEG
// 미리보기가 나오지 않게 되어 Python 판정에만 흑백 영상 조건이 들어갔고, C++ 게이트웨이는 JPEG만
// 기다리다 시간 초과(최대 35 s)로 넘어갔다 (2026-11-22 비행 기록, run/preview/last-ready.json 11-15
// 이후 미갱신).

#include <string>
#include <string_view>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {

inline constexpr std::string_view kPreviewOverlayTopic = "vision/overlay/jpeg";
inline constexpr std::string_view kPreviewGray8Topic = "vision/frame/gray8";

// 실제 Hailo 추론의 JPEG 미리보기 (기존 C++ 판정과 같다)
inline bool preview_overlay_ready(const std::string& topic, const Wire& metadata,
                                  const Bytes& payload) {
    if (topic != kPreviewOverlayTopic) {
        return false;
    }
    const bool real = python_equal(wire_get(metadata, "backend"), Wire("hailo")) ||
                      python_equal(wire_get(metadata, "backend"), Wire("hailo-sync"));
    const Wire synthetic = wire_get(metadata, "synthetic");
    return real && synthetic.is_boolean() && !synthetic.get<bool>() && payload.size() >= 2 &&
           payload.at(0) == 0xff && payload.at(1) == 0xd8;
}

// 흑백 영상: 본체 중계가 붙인 실제 카메라 표시와 크기가 맞는 픽셀 (Python gray8_ready)
inline bool preview_gray8_ready(const std::string& topic, const Wire& metadata,
                                const Bytes& payload) {
    if (topic != kPreviewGray8Topic) {
        return false;
    }
    const Wire synthetic = wire_get(metadata, "synthetic");
    const Wire stride = wire_get(metadata, "stride");
    const Wire height = wire_get(metadata, "height");
    if (!python_equal(wire_get(metadata, "schema"), Wire("argus.vision.frame.v1")) ||
        !synthetic.is_boolean() || synthetic.get<bool>() ||
        !python_equal(wire_get(metadata, "input_kind"), Wire("real_camera")) ||
        !stride.is_number_integer() || !height.is_number_integer()) {
        return false;
    }
    // Python: metadata['stride'] * metadata['height'] == len(parts[2]) > 0
    const long long pixels = stride.get<long long>() * height.get<long long>();
    return !payload.empty() && pixels == static_cast<long long>(payload.size());
}

inline bool preview_frame_ready(const std::string& topic, const Wire& metadata,
                                const Bytes& payload) {
    return preview_overlay_ready(topic, metadata, payload) ||
           preview_gray8_ready(topic, metadata, payload);
}

}  // namespace argus::gateway
