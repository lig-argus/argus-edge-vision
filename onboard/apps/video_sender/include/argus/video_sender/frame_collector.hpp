// 같은 프레임의 프레임, 탐지 결과, 추적 결과를 CollectedFrame 하나로 출력한다 (메시지 결합)
// 규약: docs/spec/process_interfaces.md 5.3, docs/design/uml/class_video.md 1절
// 입력: 프레임은 IFrameSource 의 handler, 탐지 결과와 추적 결과는 bus 구독 어댑터가 on_*() 를 호출
// 출력: subscribe() 로 등록한 handler
//
// 용어: PendingFrame 은 같은 프레임의 메시지 중 현재까지 도착한 것을 보관하는 항목이다
//
// 규칙
// - source 와 frame_id 가 같고 session_id 가 서로 다르지 않은 메시지를 같은 프레임의 메시지로
//   판단한다. session_id 가 없는 메시지(Observations, session_id 가 비어 있는 Frame 과 Tracks)는
//   source 와 frame_id 만 비교한다
// - 프레임, 탐지 결과, 추적 결과가 모두 도착하면 즉시 출력한다
// - 한 프레임의 메시지 중 최초로 도착한 것으로부터 max_wait 가 지나면, 프레임이 도착해 있는 경우
//   도착한 메시지만으로 출력하고 프레임이 도착하지 않은 경우 출력하지 않고 폐기한다
// - PendingFrame 이 max_pending 개를 초과하면 가장 오래된 PendingFrame 을 max_wait 가 지난
//   경우와 같은 방식으로 처리한다
// - 같은 source 의 출력 순서는 PendingFrame 이 생성된 순서다. 어떤 PendingFrame 을 출력할 때
//   같은 source 의 더 오래된 PendingFrame 이 남아 있으면 그것을 먼저 처리한다 (각 토픽의
//   메시지는 발행 순서대로 도착하므로 더 오래된 프레임의 나머지 메시지는 더 도착하지 않는다)
// - Frame 의 session_id 가 같은 source 의 직전 Frame 과 다르면, 그 source 의 PendingFrame 중
//   session_id 가 그 값과 다르거나 없는 것을 폐기한다. 신규 session 의 첫 Frame 보다 먼저 도착한
//   탐지 결과도 session_id 가 없으므로 폐기하며, 그 프레임은 탐지 결과 없이 출력한다 (session_id 가
//   없는 탐지 결과는 직전 session 의 것과 구분할 수 없다)
//
// 스레드: 모든 함수는 한 스레드(core::IPoller::process_events() 를 호출하는 스레드)에서 호출한다
#ifndef ARGUS_VIDEO_SENDER_FRAME_COLLECTOR_HPP_
#define ARGUS_VIDEO_SENDER_FRAME_COLLECTOR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <list>
#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "argus/core/clock.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/collected_frame.hpp"

namespace argus::video_sender {

struct FrameCollectorConfig {
    // 카메라 수신부터 tracking/tracks 도착까지의 측정값이 p50 26.3 ms 이므로
    // (process_interfaces.md 7장 3) 약 4배로 정한 초기값. tracking/tracks 가 발행되지 않는 동안에는
    // 모든 프레임이 이 시간만큼 늦게 출력된다
    static constexpr std::chrono::milliseconds kDefaultMaxWait{100};

    // 30 FPS 에서 kDefaultMaxWait 동안 도착하는 프레임 3개에, 프레임보다 먼저 도착한 탐지 결과와
    // 추적 결과의 PendingFrame 을 더한 값
    static constexpr std::size_t kDefaultMaxPending = 8;

    std::chrono::milliseconds max_wait = kDefaultMaxWait;
    std::size_t max_pending = kDefaultMaxPending;  // 1 미만이면 1 로 처리
};

class FrameCollector {
public:
    using Handler = core::HandlerList<const CollectedFrame&>::Handler;

    explicit FrameCollector(const core::IClock& clock, FrameCollectorConfig config = {});

    void on_frame(perception_api::Frame frame);
    void on_observations(perception_api::Observations observations);
    void on_tracks(perception_api::Tracks tracks);

    // bus 재연결처럼 이전에 도착한 메시지를 이후에 도착할 메시지와 같은 프레임으로 판단하면 안
    // 되는 경우에 호출한다. 보관 중인 메시지를 모두 출력하지 않고 폐기한다
    void on_session_change();

    // max_wait 가 지난 PendingFrame 을 처리한다. 메시지가 더 도착하지 않아도 프레임이 출력되도록
    // core::EventLoop::add_periodic() 으로 주기 호출한다. on_*() 도 같은 검사를 수행한다
    void process_timeouts();

    // CollectedFrame 을 출력할 때마다 handler 를 호출한다. 여러 번 등록하면 모두 호출한다.
    // handler 는 on_*() 또는 process_timeouts() 안에서 호출된다.
    // 반환된 Subscription 이 소멸하면 그 handler 는 더 호출되지 않는다
    [[nodiscard]] core::Subscription subscribe(Handler handler);

private:
    // 같은 프레임의 메시지 중 현재까지 도착한 것
    struct PendingFrame {
        std::string source;
        std::optional<std::string> session_id;  // 도착한 메시지 중 session_id 가 있는 것의 값
        std::uint64_t frame_id = 0;
        std::int64_t first_arrival_monotonic_ns = 0;
        std::optional<perception_api::Frame> frame;
        std::optional<perception_api::Observations> observations;
        std::optional<perception_api::Tracks> tracks;
    };
    // 원소는 생성된 순서다. std::list 인 이유: release_through() 가 순회 중에 중간 원소를
    // 삭제하는데, 삭제 후에도 나머지 원소의 iterator 가 유효해야 한다
    using PendingList = std::list<PendingFrame>;

    PendingFrame& find_or_add_pending(std::string_view source,
                                      std::optional<std::string_view> session_id,
                                      std::uint64_t frame_id);
    void discard_other_sessions(const perception_api::Frame& frame);
    void release_ready();
    void release_complete();
    void release_over_limit();
    void release_through(PendingList::iterator last);
    void publish(PendingFrame pending);

    const core::IClock& clock_;
    std::int64_t max_wait_ns_;
    std::size_t max_pending_;
    PendingList pending_;
    std::map<std::string, std::string, std::less<>> frame_session_by_source_;
    core::HandlerList<const CollectedFrame&> handlers_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_FRAME_COLLECTOR_HPP_
