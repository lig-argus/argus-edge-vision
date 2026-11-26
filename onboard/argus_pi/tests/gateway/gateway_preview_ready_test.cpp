// 2026-11-23 영상 준비 판정 시험 (장비·소켓 없음). Python preview_producer_supervisor.py
// ensure()의 overlay_ready, gray8_ready와 같은 결과인지 본다.
#include <cassert>
#include <iostream>

#include "argus/gateway/service/include/preview_ready.hpp"

namespace gw = argus::gateway;

namespace {

gw::Wire gray8_meta() {
    return gw::Wire::parse(R"({"schema":"argus.vision.frame.v1","synthetic":false,
        "input_kind":"real_camera","stride":640,"height":480,"frame_id":7,
        "source":"ir_sdk:/home/user/ARGUS/bin/i3_managed_capture:device=0"})");
}

}  // namespace

int main() {
    const std::string gray(gw::kPreviewGray8Topic), jpeg(gw::kPreviewOverlayTopic);
    const gw::Bytes pixels(640 * 480, 0x80);

    // 흑백 영상: 모든 조건이 맞으면 준비 완료
    assert(gw::preview_gray8_ready(gray, gray8_meta(), pixels));
    assert(gw::preview_frame_ready(gray, gray8_meta(), pixels));
    // 주제가 다르면 아니다
    assert(!gw::preview_gray8_ready(jpeg, gray8_meta(), pixels));
    // 가짜 영상, 재생 입력, 다른 형식 이름은 아니다
    {
        auto m = gray8_meta();
        m["synthetic"] = true;
        assert(!gw::preview_frame_ready(gray, m, pixels));
        m["synthetic"] = 0;  // Python `is False`: 숫자 0은 False가 아니다
        assert(!gw::preview_frame_ready(gray, m, pixels));
    }
    {
        auto m = gray8_meta();
        m["input_kind"] = "replay";
        assert(!gw::preview_frame_ready(gray, m, pixels));
    }
    {
        auto m = gray8_meta();
        m["schema"] = "argus.vision.frame.v0";
        assert(!gw::preview_frame_ready(gray, m, pixels));
        m.erase("schema");
        assert(!gw::preview_frame_ready(gray, m, pixels));
    }
    // 크기: stride * height가 픽셀 길이와 같고 0보다 커야 한다
    assert(!gw::preview_frame_ready(gray, gray8_meta(), gw::Bytes(640 * 479, 0)));
    assert(!gw::preview_frame_ready(gray, gray8_meta(), gw::Bytes{}));
    {
        auto m = gray8_meta();
        m["stride"] = 640.0;  // Python isinstance(int): 실수는 아니다
        assert(!gw::preview_frame_ready(gray, m, pixels));
        // bool은 정수로 보지 않는다. Python isinstance(True, int)는 참이라 이 경우만 C++이 더
        // 엄격하다 (생산자는 stride에 bool을 넣지 않는다).
        m["stride"] = true;
        assert(!gw::preview_frame_ready(gray, m, pixels));
        m["stride"] = 0;
        m["height"] = 0;
        assert(!gw::preview_frame_ready(gray, m, gw::Bytes{}));
    }

    // JPEG 미리보기: 기존 판정 그대로
    const gw::Bytes jpg{0xff, 0xd8, 0xff, 0xe0};
    const auto overlay = gw::Wire::parse(R"({"backend":"hailo","synthetic":false})");
    assert(gw::preview_overlay_ready(jpeg, overlay, jpg));
    assert(gw::preview_frame_ready(jpeg, overlay, jpg));
    assert(gw::preview_frame_ready(
        jpeg, gw::Wire::parse(R"({"backend":"hailo-sync","synthetic":false})"), jpg));
    assert(!gw::preview_frame_ready(
        jpeg, gw::Wire::parse(R"({"backend":"mock","synthetic":false})"), jpg));
    assert(!gw::preview_frame_ready(
        jpeg, gw::Wire::parse(R"({"backend":"hailo","synthetic":true})"), jpg));
    assert(!gw::preview_frame_ready(jpeg, overlay, gw::Bytes{0x89, 0x50}));
    // 흑백 메타데이터를 JPEG 주제로 보내도 아니다
    assert(!gw::preview_frame_ready(jpeg, gray8_meta(), pixels));

    std::cout << "PASS preview ready: jpeg overlay and gray8 frame rules match Python ensure()\n";
    return 0;
}
