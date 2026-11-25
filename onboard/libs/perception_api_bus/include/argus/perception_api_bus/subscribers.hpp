// vision/frame/gray8, perception/observations, tracking/tracks 토픽의 구독자를 만든다
// 규약: docs/spec/process_interfaces.md 5.1 (토픽 이름과 큐 길이는 argus/protocol/topics.hpp)
// 사용: 영상 송신부, 추적기, 추종 제어의 실행 파일이 받는 토픽마다 하나씩 만든다
//
// 구독자는 bus::Subscriber<T> 를 그대로 쓴다. 이 라이브러리는 토픽 이름, 메시지 타입, 큐 길이를
// 정해 만드는 일만 한다. handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른
// 스레드에서 불린다. 형식이 맞지 않는 메시지는 통신 구현이 폐기하므로 handler 까지 오지 않는다
#ifndef ARGUS_PERCEPTION_API_BUS_SUBSCRIBERS_HPP_
#define ARGUS_PERCEPTION_API_BUS_SUBSCRIBERS_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::perception_api_bus {

using BusFrameSubscriber = bus::Subscriber<perception_api::Frame>;
using BusObservationsSubscriber = bus::Subscriber<perception_api::Observations>;
using BusTracksSubscriber = bus::Subscriber<perception_api::Tracks>;

// 아래 셋은 bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)

// vision/frame/gray8. 프레임의 픽셀 버퍼는 크다. handler 는 참조로 받고, 보관하려면 복사한다
std::unique_ptr<BusFrameSubscriber> create_frame_subscriber(bus::IBus& bus);

// perception/observations
std::unique_ptr<BusObservationsSubscriber> create_observations_subscriber(bus::IBus& bus);

// tracking/tracks
std::unique_ptr<BusTracksSubscriber> create_tracks_subscriber(bus::IBus& bus);

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_SUBSCRIBERS_HPP_
