// 실제 처리 결과의 진단값만 발행한다. 제어권·표적 지정 상태를 만들지 않는다.
#pragma once

#include <cstdint>

#include "argus/common/serialization/include/native_json_io.hpp"

namespace follow {
class TrackingStatusSnapshot final {
public:
    explicit TrackingStatusSnapshot(std::int64_t begin_ns)
        : window_begin_ns_(begin_ns), next_publish_ns_(begin_ns) {}

    void record_success(std::int64_t original_sample_ns) {
        last_sample_ns_ = original_sample_ns;
        last_valid_ = true;
        ++window_frames_;
    }
    void record_failure() { last_valid_ = false; }
    bool due(std::int64_t now_ns) const { return now_ns >= next_publish_ns_; }
    NativeJson snapshot(std::int64_t now_ns, std::uint64_t generation, bool fc_fresh,
                        bool same_fc_epoch = true) {
        const auto elapsed_ns = now_ns - window_begin_ns_;
        const double fps = elapsed_ns > 0 ? window_frames_ * 1e9 / elapsed_ns : 0;
        // PiGroundProjector와 같은 원본 표본 250ms 경계다. 진단 수신 시각으로 연장하지 않는다.
        const bool cmc_valid = fc_fresh && same_fc_epoch && last_valid_ && last_sample_ns_ > 0 &&
                               now_ns >= last_sample_ns_ && now_ns - last_sample_ns_ < 250000000;
        window_frames_ = 0;
        window_begin_ns_ = now_ns;
        next_publish_ns_ = now_ns + 1000000000;
        return {{"schema", "argus.tracking.status.v1"},
                {"source", "pi-ucmc"},
                {"clock_domain", "host_monotonic"},
                {"seq", ++sequence_},
                {"published_monotonic_ns", now_ns},
                {"generation", generation},
                {"fc_fresh", fc_fresh},
                {"cmc_valid", cmc_valid},
                {"fps", fps}};
    }

private:
    std::int64_t window_begin_ns_{0}, next_publish_ns_{0}, last_sample_ns_{0};
    std::uint64_t sequence_{0}, window_frames_{0};
    bool last_valid_{false};
};
}  // namespace follow
