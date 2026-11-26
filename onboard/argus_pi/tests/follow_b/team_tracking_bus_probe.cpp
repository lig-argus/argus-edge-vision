// 팀 ZmqBus 구독자로 추적기 출력을 받는 시험 도구 (2026-11-25 batch 2, 장비 없음).
// 사용: TeamTrackingBusProbe <엔드포인트> <tracks 수> <status 수> <제한 ms>
// 고치지 않은 팀 ZmqBus(third_party/argus_onboard)와 Pi 변환 표(pi_codec_table)로 tracking/tracks,
// tracking/status 를 구독하고, 받은 메시지마다 한 줄 JSON 을 낸다:
//   {"topic": ..., "sequence": 팀 seq (MessageInfo.sequence), "value": Pi 값}
// 마지막 줄은 {"done": true, "tracks": n, "status": n, "decode_failures": n}. 수를 채우면 0, 제한
// 시간 안에 못 채우거나 해석 실패가 있으면 1. team_tracking_bus_test.py 가 실행한다.
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <zmq.hpp>

#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/transport/team_wire/include/pi_codec_table.hpp"
#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus_msg/pi_tracking_status.pb.h"
#include "argus_msg/tracking_tracks.pb.h"

namespace {

using argus::team_wire::PbJson;
using Tracks = PbJson<argus::msg::TrackingTracks>;
using Status = PbJson<argus::msg::PiTrackingStatus>;

void emit(std::string_view topic, const follow::NativeJson& value, std::uint64_t sequence) {
    follow::NativeJson line = follow::NativeJson::object();
    line["topic"] = topic;
    line["sequence"] = sequence;
    line["value"] = value;
    std::cout << line.dump() << '\n' << std::flush;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 5) {
            std::cerr << "usage: TeamTrackingBusProbe <endpoint> <tracks> <status> <timeout_ms>\n";
            return 2;
        }
        const std::string endpoint = argv[1];
        const long want_tracks = std::stol(argv[2]);
        const long want_status = std::stol(argv[3]);
        const long timeout_ms = std::stol(argv[4]);

        zmq::context_t context;
        argus::core::SteadyClock clock;
        argus::bus_zmq::ZmqPoller poller{clock};
        auto stats = std::make_shared<argus::team_wire::CodecStats>();
        argus::bus_zmq::ZmqBus bus(context, poller, clock,
                                   argus::bus_zmq::ZmqEndpointTable{
                                       {std::string(argus::team_wire::kTrackingTracks), endpoint},
                                       {std::string(argus::team_wire::kTrackingStatus), endpoint}},
                                   argus::team_wire::pi_codec_table(stats));
        auto tracks = argus::bus::make_subscriber<Tracks>(
            bus, argus::team_wire::kTrackingTracks,
            argus::team_wire::pb_json_type<argus::msg::TrackingTracks>(),
            argus::team_wire::kTrackingTracksQueueDepth);
        auto status = argus::bus::make_subscriber<Status>(
            bus, argus::team_wire::kTrackingStatus,
            argus::team_wire::pb_json_type<argus::msg::PiTrackingStatus>(),
            argus::team_wire::kTrackingStatusQueueDepth);
        if (!tracks || !status) {
            std::cerr << "team ZmqBus subscriber unavailable\n";
            return 1;
        }
        long got_tracks = 0;
        long got_status = 0;
        auto tracks_subscription =
            tracks->subscribe([&](const Tracks& message, const argus::bus::MessageInfo& info) {
                ++got_tracks;
                emit(argus::team_wire::kTrackingTracks, message.value, info.sequence);
            });
        auto status_subscription =
            status->subscribe([&](const Status& message, const argus::bus::MessageInfo& info) {
                ++got_status;
                emit(argus::team_wire::kTrackingStatus, message.value, info.sequence);
            });
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while ((got_tracks < want_tracks || got_status < want_status) &&
               std::chrono::steady_clock::now() < deadline) {
            poller.process_events(std::chrono::milliseconds(20), std::chrono::milliseconds(50));
        }
        const auto failures = stats->decode_failures.load();
        follow::NativeJson done = follow::NativeJson::object();
        done["done"] = true;
        done["tracks"] = got_tracks;
        done["status"] = got_status;
        done["decode_failures"] = failures;
        std::cout << done.dump() << '\n' << std::flush;
        const bool complete = got_tracks >= want_tracks && got_status >= want_status;
        return complete && failures == 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "TeamTrackingBusProbe: " << error.what() << '\n';
        return 1;
    }
}
