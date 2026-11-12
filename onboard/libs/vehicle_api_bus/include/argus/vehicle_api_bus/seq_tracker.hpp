// 토픽 하나에서 받은 메시지의 seq 로 손실된 메시지 수를 센다
// 규약: docs/spec/process_interfaces.md 3장 (seq 는 발행마다 1 증가)
// 사용: vehicle_api_bus 의 구독 클래스
#ifndef ARGUS_VEHICLE_API_BUS_SEQ_TRACKER_HPP_
#define ARGUS_VEHICLE_API_BUS_SEQ_TRACKER_HPP_

#include <cstdint>
#include <optional>

namespace argus::vehicle_api_bus {

class SeqTracker {
public:
    // 받은 메시지의 seq 를 알린다. 직전 seq 보다 2 이상 크면 그 사이의 수만큼 손실로 센다.
    // 직전 seq 이하이면 발행자가 다시 시작한 것으로 보고 손실로 세지 않는다
    void observe(std::uint64_t seq) {
        if (last_seq_ && seq > *last_seq_) {
            lost_count_ += seq - *last_seq_ - 1;
        }
        last_seq_ = seq;
    }

    // 처음 받은 메시지 이후에 손실된 메시지 수. 처음 받기 전에 발행된 메시지는 세지 않는다
    std::uint64_t lost_count() const { return lost_count_; }

private:
    std::optional<std::uint64_t> last_seq_;
    std::uint64_t lost_count_ = 0;
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SEQ_TRACKER_HPP_
