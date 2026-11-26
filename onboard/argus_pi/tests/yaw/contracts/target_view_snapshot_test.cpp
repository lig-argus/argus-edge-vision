// 1개 대상/여러 객체의 ID 계약과 frame/epoch 일관성. GUI/FC 없이 기존 매니저를 사용한다.
#include "argus/diagnostics/tracking/include/target_view_snapshot.hpp"

#include <cassert>
#include <msgpack.hpp>

#include "argus/diagnostics/tracking/include/target_view_codec.hpp"
using namespace follow;
using namespace std::chrono_literals;
TracksV1 frame(std::uint64_t number, Clock::time_point now, std::uint64_t generation = 1,
               std::int64_t selected_id = 7, bool competitor = true) {
    TracksV1 f;
    f.source = "contract-camera";
    f.frame_id = number;
    f.generation = generation;
    f.sample_monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "host_receive_test";
    f.image_width = 640;
    f.image_height = 480;
    f.tracks.push_back({selected_id, 0, "vehicle", .9, 100, 150, 180, 220, "CONFIRMED", true,
                        f.sample_monotonic_ns});
    if (competitor) {
        f.tracks.push_back(
            {8, 0, "vehicle", .9, 300, 150, 380, 220, "CONFIRMED", true, f.sample_monotonic_ns});
    }
    return f;
}
int main() {
    TargetSelectorConfig cfg;
    cfg.recovery.enabled = true;
    cfg.require_local_monotonic_sample = true;
    TargetSelector selector(cfg);
    Clock::time_point now{10s};
    auto f = frame(1, now);
    assert(selector.update(f, now));
    auto v = build_target_view_snapshot(selector.snapshot(now), now);
    assert(v.frame_available && v.objects.size() == 2 && !v.objects[0].target_id &&
           !v.objects[1].target_id);
    assert(selector.execute({"select", TargetCommandAction::Select, 1, 7}, now).accepted);
    v = build_target_view_snapshot(selector.snapshot(now), now);
    const auto target = v.selection.mission_target_id;
    assert(target > 0 && v.objects[0].target_id == target && v.objects[0].is_selected &&
           !v.objects[0].binding_confirmed);
    assert(!v.objects[1].target_id);
    assert(selector.execute({"start", TargetCommandAction::Start, 0, 0}, now).accepted);
    v = build_target_view_snapshot(selector.snapshot(now), now);
    assert(v.objects[0].binding_confirmed);
    assert(v.selection.latest_frame->tracks[0].class_id == 0 &&
           v.selection.latest_frame->tracks[0].track_id == 7);
    const auto packed = encode_target_view(v);
    auto decoded = msgpack::unpack(packed.data(), packed.size());
    assert(decoded.get().type == msgpack::type::MAP && decoded.get().via.map.size == 15);
    // 새 세대를 가진 같은 숫자ID를 기존 target으로 보이지 않게 한다. 3회의 실제 확인 뒤만 연결한다.
    for (int i = 1; i <= 3; ++i) {
        now += 50ms;
        f = frame(1 + i, now, 2, 19, false);
        assert(selector.update(f, now));
        v = build_target_view_snapshot(selector.snapshot(now), now);
        if (i < 3) {
            assert(!v.objects[0].target_id && !v.objects[0].binding_confirmed);
        }
    }
    assert(v.selection.mission_target_id == target && v.objects[0].target_id == target &&
           v.objects[0].binding_confirmed);
    assert(v.selection.latest_frame->generation == 2 &&
           v.selection.latest_frame->tracks[0].track_id == 19);
    // 같은 잠금에서 나온 snapshot만 보유한다. Stop 뒤 새 snapshot은 기존 target ID를 재사용하지
    // 않는다.
    const auto old_epoch = v.selection.command_epoch;
    assert(selector.execute({"stop", TargetCommandAction::Stop, 0, 0}, now).accepted);
    auto stopped = build_target_view_snapshot(selector.snapshot(now), now);
    assert(stopped.selection.command_epoch > old_epoch && !stopped.objects[0].target_id);
    assert(v.objects[0].target_id == target && v.selection.command_epoch == old_epoch);
    // 임의 DTO나 이미 만료된 표본도 새 현재 박스/확정 ID로 표시하지 않는다.
    auto stale = build_target_view_snapshot(v.selection, now + 300ms);
    assert(!stale.frame_available && stale.objects.empty());
    auto lost = v.selection;
    lost.target_phase = TargetPhase::Lost;
    lost.fresh_real_detection = false;
    auto lv = build_target_view_snapshot(lost, now);
    assert(lv.selection.mission_target_id == target && !lv.objects[0].target_id &&
           !lv.objects[0].binding_confirmed);
    auto foreign = v.selection;
    foreign.latest_frame->clock_domain = "other_host";
    assert(!build_target_view_snapshot(foreign, now).frame_available);
    return 0;
}
