#pragma once

#include <cstdint>
#include <deque>
#include <utility>

namespace follow {

// 2026-11-12: 시계 정렬 실패가 성공과 섞여 이어질 때도 정렬 추정을 다시 잡는다.
// 기존 규칙(실패만 3초 연속)은 가끔 성공하는 프레임 때문에 계속 초기화되어, 16:13 실내 비행에서
// 프레임 대부분이 약 15초 동안 거부됐다. 최근 1초 FC 투영 시도 중 실패가 8회 이상이고 절반
// 이상이면 다시 잡는다. 다시 잡은 뒤 2초는 새 추정이 준비되도록 판단하지 않는다.
class AlignmentFailureWindow {
public:
    static constexpr std::int64_t kWindowNs = 1000000000LL;
    static constexpr std::int64_t kCooldownNs = 2000000000LL;
    static constexpr int kMinimumFailures = 8;

    // FC 투영 시도 하나를 기록한다. 다시 잡아야 하면 true.
    bool record(std::int64_t now_ns, bool alignment_failure) {
        attempts_.emplace_back(now_ns, alignment_failure);
        failures_ += alignment_failure ? 1 : 0;
        while (!attempts_.empty() && now_ns - attempts_.front().first > kWindowNs) {
            failures_ -= attempts_.front().second ? 1 : 0;
            attempts_.pop_front();
        }
        if (last_reanchor_ns_ != 0 && now_ns - last_reanchor_ns_ < kCooldownNs) {
            return false;
        }
        const auto total = static_cast<int>(attempts_.size());
        return failures_ >= kMinimumFailures && failures_ * 2 >= total;
    }

    // 다시 잡은 시각을 남기고 창을 비운다.
    void reanchored(std::int64_t now_ns) {
        last_reanchor_ns_ = now_ns;
        attempts_.clear();
        failures_ = 0;
    }

    int failures() const { return failures_; }
    int attempts() const { return static_cast<int>(attempts_.size()); }

private:
    std::deque<std::pair<std::int64_t, bool>> attempts_;
    int failures_{0};
    std::int64_t last_reanchor_ns_{0};
};

}  // namespace follow
