// 게이트웨이 5단계 단위 시험: 메인 GUI 연결부의 실제 소켓과 MAVLink 해석기.
// UDP 루프백(외부 수신·송신, 라우터 왕복), ZMQ 추적 SUB 주제 필터, 프레임 나누기·잡음 건너뛰기.
// Python 원본과의 대량 비교는 tests/gateway/differential/stage5a_diff.py, stage5b_diff.py가 맡는다.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zmq.h>

#include <array>
#include <chrono>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>

#include "argus/gateway/bridge/include/gcs_mavlink.hpp"
#include "argus/gateway/bridge/include/udp_sockets.hpp"

namespace gw = argus::gateway;

namespace {

int& failures() {
    static int count = 0;
    return count;
}

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

// Linux 임시 포트 범위(32768~)와 겹치지 않는 20000~29999에서 고른다.
int test_port(int offset) { return 20000 + (((::getpid() * 11) + offset) % 10000); }

gw::Bytes text(const std::string& value) { return {value.begin(), value.end()}; }

sockaddr_in loopback(int port) {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    return address;
}

const sockaddr* as_sockaddr(const sockaddr_in& address) {
    return reinterpret_cast<const sockaddr*>(&address);  // NOLINT(*-reinterpret-cast): POSIX API
}

int bound_udp(int port) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        throw std::runtime_error("UDP 소켓 생성 실패");
    }
    const sockaddr_in address = loopback(port);
    if (::bind(fd, as_sockaddr(address), sizeof(address)) != 0) {
        ::close(fd);
        throw std::runtime_error("UDP bind 실패: " + std::to_string(port));
    }
    return fd;
}

int local_port(int fd) {
    sockaddr_in address{};
    socklen_t length = sizeof(address);
    // POSIX API는 sockaddr*를 받는다
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&address),  // NOLINT(*-reinterpret-cast)
                  &length);
    return ntohs(address.sin_port);
}

std::string receive_text(int fd, int timeout_ms) {
    pollfd item{fd, POLLIN, 0};
    if (::poll(&item, 1, timeout_ms) <= 0) {
        return "";
    }
    std::array<char, 256> buffer{};
    const ssize_t got = ::recv(fd, buffer.data(), buffer.size(), 0);
    return got > 0 ? std::string(buffer.data(), static_cast<std::size_t>(got)) : "";
}

template <typename T, typename F>
// 작은 람다를 값으로 받아 여러 번 부른다
std::optional<T> wait_for(F poll) {  // NOLINT(performance-unnecessary-value-param)
    for (int i = 0; i < 200; ++i) {
        if (auto value = poll()) {
            return value;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::nullopt;
}

void sockets() {
    const int external_port = test_port(1);
    const int router_port = test_port(2);
    const std::string tracks = "tcp://127.0.0.1:" + std::to_string(test_port(3));
    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context, ZMQ_PUB);
    const int linger = 0;
    zmq_setsockopt(publisher, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_bind(publisher, tracks.c_str());
    const int router_server = bound_udp(router_port);
    const int gui = bound_udp(0);
    {
        gw::UdpBridgeSockets bridge(external_port, router_port, tracks);
        // 외부 수신: 보낸 주소 그대로
        const sockaddr_in external = loopback(external_port);
        ::sendto(gui, "abc", 3, 0, as_sockaddr(external), sizeof(external));
        bridge.wait_readable(1.0);
        const auto incoming = wait_for<std::pair<gw::Bytes, gw::GcsAddress>>(
            [&] { return bridge.receive_external(); });
        check(incoming && incoming->first == text("abc") && incoming->second.host == "127.0.0.1" &&
                  incoming->second.port == local_port(gui),
              "외부 수신과 보낸 주소");
        check(!bridge.receive_external(), "읽을 것이 없으면 기다리지 않음");
        // 외부 송신
        bridge.send_external(text("xyz"), gw::GcsAddress{"127.0.0.1", local_port(gui)});
        check(receive_text(gui, 2000) == "xyz", "외부 송신");
        // 라우터 왕복 (임시 포트에서 라우터 포트로 연결)
        bridge.send_router(text("r1"));
        check(receive_text(router_server, 2000) == "r1", "라우터 송신");
        const sockaddr_in back = loopback(bridge.router_local_port());
        ::sendto(router_server, "r2", 2, 0, as_sockaddr(back), sizeof(back));
        const auto routed = wait_for<gw::Bytes>([&] { return bridge.receive_router(); });
        check(routed && *routed == text("r2"), "라우터 수신");
        // 추적 SUB: 늦게 붙는 구독자를 위해 여러 번 보내고, 다른 주제는 받지 않는다
        std::optional<std::vector<gw::Bytes>> track;
        bool other = false;
        for (int i = 0; i < 200 && !track; ++i) {
            zmq_send(publisher, "other/topic", 11, ZMQ_SNDMORE);
            zmq_send(publisher, "x", 1, 0);
            zmq_send(publisher, "tracking/tracks", 15, ZMQ_SNDMORE);
            zmq_send(publisher, "payload", 7, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            while (auto parts = bridge.receive_tracks()) {
                other = other ||
                        std::string(parts->at(0).begin(), parts->at(0).end()) != "tracking/tracks";
                if (parts->size() == 2) {
                    track = parts;
                }
            }
        }
        check(track && track->at(1) == text("payload"), "추적 메시지 수신");
        check(!other, "구독하지 않은 주제 거부");
    }
    ::close(gui);
    ::close(router_server);
    zmq_close(publisher);
    zmq_ctx_term(context);
}

void parser() {
    // 메인 GUI heartbeat (현 dialect 표시)
    const gw::Bytes payload = gw::heartbeat_payload(6, 8, 0, 0x41524732, 4);
    const gw::Bytes frame = gw::mavlink2_frame(7, 250, 190, gw::gcs_ids::kHeartbeat,
                                               gw::gcs_ids::kHeartbeatCrc, payload);
    gw::GcsParser parser;
    const auto split = static_cast<std::ptrdiff_t>(frame.size() / 2);
    check(parser.parse(gw::Bytes(frame.begin(), std::next(frame.begin(), split))).empty(),
          "반쪽 프레임은 기다림");
    const auto messages = parser.parse(gw::Bytes(std::next(frame.begin(), split), frame.end()));
    check(messages.size() == 1 && messages.at(0).kind == "HEARTBEAT" &&
              messages.at(0).system == 250 && messages.at(0).component == 190 &&
              messages.at(0).seq == 7 && messages.at(0).frame == frame &&
              messages.at(0).fields.at("custom_mode") == 0x41524732 &&
              messages.at(0).fields.at("type") == 6,
          "나눠 온 heartbeat 해석");
    // 앞의 잡음은 건너뛰고, CRC가 틀린 프레임은 버린다
    gw::Bytes noisy = {0x01, 0x02, 0x03};
    noisy.insert(noisy.end(), frame.begin(), frame.end());
    gw::Bytes broken = frame;
    broken.back() ^= 0xffU;
    noisy.insert(noisy.end(), broken.begin(), broken.end());
    noisy.insert(noisy.end(), frame.begin(), frame.end());
    gw::GcsParser second;
    check(second.parse(noisy).size() == 2, "잡음·CRC 오류 건너뛰기");
    // 구 dialect ARGUS_STATUS(45000, CRC 92)도 해석한다
    const gw::Bytes legacy =
        gw::mavlink2_frame(1, 250, 190, gw::gcs_ids::kStatus, gw::gcs_ids::kLegacyStatusCrc,
                           gw::legacy_status_payload(1, 0, 0, 0, 0, 0, 0));
    gw::GcsParser third;
    const auto status = third.parse(legacy);
    check(status.size() == 1 && status.at(0).kind == "ARGUS_STATUS", "구 dialect 메시지");
    // Python struct.pack과 같은 오류
    bool range = false;
    try {
        gw::PayloadWriter writer;
        writer.u8(300);
    } catch (const gw::StructError& error) {
        range = std::string(error.what()) == "'B' format requires 0 <= number <= 255";
    }
    check(range, "struct 범위 오류 문구");
}

}  // namespace

int main() {
    try {
        sockets();
        parser();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayStage5Test 통과\n";
    return 0;
}
