#include "argus/bus_zmq/zmq_bus.hpp"

#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <zmq.hpp>

#include "argus/bus/message_type.hpp"
#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_clock.hpp"

namespace argus::bus_zmq {
namespace {

using bus::MessageInfo;
using std::chrono::milliseconds;

constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr milliseconds kBudget{1000};
constexpr int kDepth = 4;

// 테스트가 주고받는 값. 토픽, 요청, 응답에 모두 쓴다
struct Text {
    std::string text;
    Bytes binary;
};

// 변환 표에 넣지 않는 타입
struct Unregistered {
    int value = 0;
};

// encode 가 실패하는 값
const char* const kUnencodable = "unencodable";

bus::MessageType text_type() { return bus::message_type<Text>("test.Text"); }

Bytes bytes(const std::string& text) { return {text.begin(), text.end()}; }
std::string text_of(const Bytes& data) { return {data.begin(), data.end()}; }

// 본문 "<번호>|<text>". 번호나 구분자가 없는 본문은 decode 가 실패한다
ZmqCodecTable codecs() {
    ZmqCodecTable table;
    add_zmq_codec<Text>(
        table,
        [](const Text& value, std::uint64_t sequence, Bytes& body, Bytes& binary) {
            if (value.text == kUnencodable) {
                return false;
            }
            body = bytes(std::to_string(sequence) + "|" + value.text);
            binary = value.binary;
            return true;
        },
        [](const Bytes& body, const Bytes& binary, Text& value, std::uint64_t& sequence) {
            const std::string received = text_of(body);
            const std::size_t separator = received.find('|');
            if (separator == std::string::npos || separator == 0 ||
                received.find_first_not_of("0123456789") != separator) {
                return false;
            }
            sequence = std::stoull(received.substr(0, separator));
            value.text = received.substr(separator + 1);
            value.binary = binary;
            return true;
        });
    return table;
}

// 본문이 text 그대로인 변환 표. codecs() 의 decode 가 실패하는 바이트를 보내는 데 쓴다
ZmqCodecTable plain_codecs() {
    ZmqCodecTable table;
    add_zmq_codec<Text>(
        table,
        [](const Text& value, std::uint64_t /*sequence*/, Bytes& body, Bytes& binary) {
            body = bytes(value.text);
            binary = value.binary;
            return true;
        },
        [](const Bytes& body, const Bytes& binary, Text& value, std::uint64_t& sequence) {
            sequence = 0;
            value.text = text_of(body);
            value.binary = binary;
            return true;
        });
    return table;
}

// 비어 있는 TCP 포트 번호. 소켓을 닫은 뒤 다른 프로세스가 먼저 잡을 수 있으나 테스트에서는 무시한다
int free_tcp_port() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t length = sizeof(address);
    int port = 0;
    if (fd >= 0 && ::bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0 &&
        ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0) {
        port = ntohs(address.sin_port);
    }
    if (fd >= 0) {
        ::close(fd);
    }
    return port;
}

// 시각은 FakeClock 으로 정한다. 요청의 timeout 은 테스트가 시각을 옮길 때만 지난다
class ZmqBusTest : public ::testing::Test {
protected:
    using TextPublisher = bus::Publisher<Text>;
    using TextSubscriber = bus::Subscriber<Text>;
    using TextRequester = bus::Requester<Text, Text>;
    using TextReplier = bus::Replier<Text, Text>;

    // 테스트마다 다른 inproc 엔드포인트를 만든다 (같은 context 안에서 이름이 겹치지 않게)
    std::string endpoint(const std::string& label) const {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        return std::string("inproc://") + info->name() + "_" + label;
    }

    static std::unique_ptr<TextPublisher> publisher(ZmqBus& bus, const std::string& topic,
                                                    int depth = kDepth) {
        return bus::make_publisher<Text>(bus, topic, text_type(), depth);
    }
    static std::unique_ptr<TextSubscriber> subscriber(ZmqBus& bus, const std::string& topic,
                                                      int depth = kDepth) {
        return bus::make_subscriber<Text>(bus, topic, text_type(), depth);
    }
    static std::unique_ptr<TextRequester> requester(ZmqBus& bus, const std::string& name) {
        return bus::make_requester<Text, Text>(bus, name, text_type(), text_type());
    }
    static std::unique_ptr<TextReplier> replier(ZmqBus& bus, const std::string& name) {
        return bus::make_replier<Text, Text>(bus, name, text_type(), text_type());
    }

    // condition 이 참이 될 때까지 이벤트를 처리한다. 끝까지 참이 되지 않으면 false
    bool pump_until(const std::function<bool()>& condition) {
        for (int i = 0; i < 400 && !condition(); ++i) {
            poller_.process_events(milliseconds{5}, kBudget);
        }
        return condition();
    }

    // 정해진 횟수만큼 이벤트를 처리한다 (오지 않아야 하는 메시지를 기다릴 때)
    void pump_rounds(int rounds) {
        for (int i = 0; i < rounds; ++i) {
            poller_.process_events(milliseconds{5}, kBudget);
        }
    }

    // PUB/SUB 는 연결 직후의 메시지를 잃는다. 구독자가 받기 시작할 때까지 연결 확인용 메시지를
    // 보내고, 남은 확인용 메시지를 모두 흘려보낸 뒤 반환한다
    bool wait_until_connected(TextPublisher& pub, TextSubscriber& sub) {
        int received = 0;
        const core::Subscription probe =
            sub.subscribe([&](const Text&, const MessageInfo&) { ++received; });
        for (int i = 0; i < 400 && received == 0; ++i) {
            pub.publish(Text{"probe", {}});
            poller_.process_events(milliseconds{5}, kBudget);
        }
        int last = -1;
        while (last != received) {
            last = received;
            pump_rounds(4);
        }
        return received > 0;
    }

    // 받은 text 를 기록하는 handler
    static TextSubscriber::Handler record_text_into(std::vector<std::string>& received) {
        return [&received](const Text& message, const MessageInfo&) {
            received.push_back(message.text);
        };
    }

    zmq::context_t context_;
    core_testing::FakeClock clock_;
    ZmqPoller poller_{clock_};
};

// ------------------------------------------------------------------------------------------ 토픽

TEST_F(ZmqBusTest, DeliversValueToSubscriber) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    const auto sub = subscriber(bus, "fc/state");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    EXPECT_EQ(pub->topic(), "fc/state");
    EXPECT_EQ(sub->topic(), "fc/state");
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<Text> received;
    const core::Subscription subscription = sub->subscribe(
        [&](const Text& message, const MessageInfo&) { received.push_back(message); });
    EXPECT_TRUE(pub->publish(Text{"body", {}}));
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].text, "body");
    EXPECT_TRUE(received[0].binary.empty());
}

TEST_F(ZmqBusTest, DeliversBinaryWhenPresent) {
    ZmqBus bus(context_, poller_, clock_, {{"vision/frame/gray8", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "vision/frame/gray8");
    const auto sub = subscriber(bus, "vision/frame/gray8");
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<Text> received;
    const core::Subscription subscription = sub->subscribe(
        [&](const Text& message, const MessageInfo&) { received.push_back(message); });
    const Bytes pixels(1024, 0x7F);
    EXPECT_TRUE(pub->publish(Text{"header", pixels}));
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));

    EXPECT_EQ(received[0].text, "header");
    EXPECT_EQ(received[0].binary, pixels);
}

// 전송 바이트 (docs/spec/process_interfaces.md 3장): [0] 이름, [1] 본문, 필요하면 [2] 바이너리.
// ZmqBus 를 거치지 않는 SUB 소켓으로 받아 프레임을 직접 확인한다
TEST_F(ZmqBusTest, SendsNameBodyAndOptionalBinaryFrames) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, plain_codecs());
    const auto pub = publisher(bus, "fc/state");
    ASSERT_NE(pub, nullptr);
    zmq::socket_t raw(context_, zmq::socket_type::sub);
    raw.set(zmq::sockopt::subscribe, "");
    raw.set(zmq::sockopt::rcvtimeo, 5);
    raw.connect(endpoint("pub"));

    // 메시지 하나의 프레임들을 읽는다. 5 ms 안에 오지 않으면 빈 목록
    const auto receive = [&raw] {
        std::vector<std::string> frames;
        zmq::message_t frame;
        while (raw.recv(frame).has_value()) {
            frames.push_back(frame.to_string());
            if (!frame.more()) {
                break;
            }
        }
        return frames;
    };
    // 연결이 끝날 때까지 바이너리가 없는 메시지를 보낸다
    std::vector<std::string> without_binary;
    for (int i = 0; i < 400 && without_binary.empty(); ++i) {
        EXPECT_TRUE(pub->publish(Text{"body", {}}));
        without_binary = receive();
    }
    while (!receive().empty()) {
    }
    EXPECT_TRUE(pub->publish(Text{"header", {1, 2, 3}}));
    const std::vector<std::string> with_binary = receive();

    EXPECT_EQ(without_binary, (std::vector<std::string>{"fc/state", "body"}));
    EXPECT_EQ(with_binary, (std::vector<std::string>{"fc/state", "header", "\x01\x02\x03"}));
}

TEST_F(ZmqBusTest, SequenceStartsAtOneAndIncreasesByOneForEachPublish) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state", 64);
    const auto sub = subscriber(bus, "fc/state", 64);
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription = sub->subscribe(
        [&](const Text&, const MessageInfo& info) { sequences.push_back(info.sequence); });

    // 연결 전에 발행한 메시지는 도착하지 않으므로, 받은 번호가 1 씩 이어지는지와 발행 횟수와
    // 같은 번호까지 오는지를 확인한다
    std::uint64_t published = 0;
    for (int i = 0; i < 400 && sequences.size() < 3; ++i) {
        EXPECT_TRUE(pub->publish(Text{"m", {}}));
        ++published;
        pump_rounds(1);
    }
    ASSERT_TRUE(pump_until([&] { return !sequences.empty() && sequences.back() == published; }));

    ASSERT_GE(sequences.size(), 3U);
    for (std::size_t i = 1; i < sequences.size(); ++i) {
        EXPECT_EQ(sequences[i], sequences[i - 1] + 1);
    }
}

TEST_F(ZmqBusTest, EachPublisherCountsItsOwnSequence) {
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/attitude", endpoint("pub")}, {"fc/state", endpoint("pub")}}, codecs());
    const auto attitude_publisher = publisher(bus, "fc/attitude");
    const auto state_publisher = publisher(bus, "fc/state");
    const auto state_subscriber = subscriber(bus, "fc/state");
    const auto attitude_subscriber = subscriber(bus, "fc/attitude");
    ASSERT_TRUE(wait_until_connected(*attitude_publisher, *attitude_subscriber));

    // fc/attitude 를 여러 번 발행한 뒤에도 fc/state 의 첫 발행은 1 이다
    std::vector<std::uint64_t> sequences;
    const core::Subscription subscription = state_subscriber->subscribe(
        [&](const Text&, const MessageInfo& info) { sequences.push_back(info.sequence); });
    EXPECT_TRUE(state_publisher->publish(Text{"s", {}}));
    ASSERT_TRUE(pump_until([&] { return !sequences.empty(); }));

    EXPECT_EQ(sequences, std::vector<std::uint64_t>{1});
}

TEST_F(ZmqBusTest, DropsMessageThatCannotBeDecoded) {
    // 보내는 bus 는 번호 없는 본문을 보내고, 받는 bus 의 decode 는 그 본문을 거부한다
    ZmqBus sender(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, plain_codecs());
    ZmqBus receiver(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(sender, "fc/state");
    const auto sub = subscriber(receiver, "fc/state");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    std::vector<std::string> received;
    const core::Subscription subscription = sub->subscribe(record_text_into(received));

    // "7|valid" 는 받는 쪽 형식에 맞는 본문이다. 이 메시지가 도착하면 연결된 것이다
    for (int i = 0; i < 400 && received.empty(); ++i) {
        EXPECT_TRUE(pub->publish(Text{"broken", {}}));
        EXPECT_TRUE(pub->publish(Text{"7|valid", {}}));
        pump_rounds(1);
    }
    pump_rounds(4);

    ASSERT_FALSE(received.empty());
    for (const std::string& text : received) {
        EXPECT_EQ(text, "valid");
    }
}

TEST_F(ZmqBusTest, PublishFailsWhenValueCannotBeEncoded) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    ASSERT_NE(pub, nullptr);

    EXPECT_FALSE(pub->publish(Text{kUnencodable, {}}));
    EXPECT_TRUE(pub->publish(Text{"ok", {}}));
}

TEST_F(ZmqBusTest, DeliversOverTcp) {
    const int port = free_tcp_port();
    ASSERT_GT(port, 0);
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", "tcp://127.0.0.1:" + std::to_string(port)}},
               codecs());
    const auto pub = publisher(bus, "fc/state");
    const auto sub = subscriber(bus, "fc/state");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<std::string> received;
    const core::Subscription subscription = sub->subscribe(record_text_into(received));
    EXPECT_TRUE(pub->publish(Text{"over tcp", {}}));
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));

    EXPECT_EQ(received, std::vector<std::string>{"over tcp"});
}

TEST_F(ZmqBusTest, DoesNotDeliverTopicThatOnlySharesPrefix) {
    // ZeroMQ 의 구독 필터로는 "fc/state" 구독자가 "fc/state_extra" 도 받는다
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/state", endpoint("pub")}, {"fc/state_extra", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    const auto other_publisher = publisher(bus, "fc/state_extra");
    const auto sub = subscriber(bus, "fc/state");
    ASSERT_NE(other_publisher, nullptr);
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<std::string> received;
    const core::Subscription subscription = sub->subscribe(record_text_into(received));
    EXPECT_TRUE(other_publisher->publish(Text{"other", {}}));
    EXPECT_TRUE(pub->publish(Text{"mine", {}}));
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));
    pump_rounds(4);

    EXPECT_EQ(received, std::vector<std::string>{"mine"});
}

TEST_F(ZmqBusTest, TopicsOnOneEndpointReachTheirOwnSubscribers) {
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/attitude", endpoint("pub")}, {"fc/state", endpoint("pub")}}, codecs());
    const auto attitude_publisher = publisher(bus, "fc/attitude");
    const auto state_publisher = publisher(bus, "fc/state");
    const auto attitude_subscriber = subscriber(bus, "fc/attitude");
    const auto state_subscriber = subscriber(bus, "fc/state");
    ASSERT_TRUE(wait_until_connected(*attitude_publisher, *attitude_subscriber));
    ASSERT_TRUE(wait_until_connected(*state_publisher, *state_subscriber));

    std::vector<std::string> attitude;
    std::vector<std::string> state;
    const core::Subscription attitude_subscription =
        attitude_subscriber->subscribe(record_text_into(attitude));
    const core::Subscription state_subscription =
        state_subscriber->subscribe(record_text_into(state));
    EXPECT_TRUE(attitude_publisher->publish(Text{"a", {}}));
    EXPECT_TRUE(state_publisher->publish(Text{"s", {}}));
    ASSERT_TRUE(pump_until([&] { return !attitude.empty() && !state.empty(); }));
    pump_rounds(4);

    EXPECT_EQ(attitude, std::vector<std::string>{"a"});
    EXPECT_EQ(state, std::vector<std::string>{"s"});
}

TEST_F(ZmqBusTest, AcceptsLargerQueueDepthOnSharedEndpoint) {
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/attitude", endpoint("pub")}, {"fc/state", endpoint("pub")}}, codecs());
    const auto small = publisher(bus, "fc/attitude", 1);
    const auto large = publisher(bus, "fc/state", 8);  // 다시 bind 한다
    ASSERT_NE(small, nullptr);
    ASSERT_NE(large, nullptr);
    const auto sub = subscriber(bus, "fc/attitude");
    ASSERT_TRUE(wait_until_connected(*small, *sub));

    std::vector<std::string> received;
    const core::Subscription subscription = sub->subscribe(record_text_into(received));
    EXPECT_TRUE(small->publish(Text{"after rebind", {}}));
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));

    EXPECT_EQ(received, std::vector<std::string>{"after rebind"});
}

TEST_F(ZmqBusTest, DeliversOnlyLatestQueueDepthMessagesWhenSubscriberFallsBehind) {
    constexpr int kPublisherDepth = 64;
    constexpr int kPublished = 20;
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state", kPublisherDepth);
    const auto sub = subscriber(bus, "fc/state");
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<std::string> received;
    const core::Subscription subscription = sub->subscribe(record_text_into(received));
    // 구독자가 이벤트를 처리하지 않는 동안 queue_depth 보다 많이 발행한다
    for (int i = 0; i < kPublished; ++i) {
        ASSERT_TRUE(pub->publish(Text{std::to_string(i), {}}));
    }
    ASSERT_TRUE(pump_until([&] { return !received.empty(); }));
    pump_rounds(4);

    EXPECT_EQ(received, (std::vector<std::string>{"16", "17", "18", "19"}));
}

TEST_F(ZmqBusTest, PublishSucceedsWithoutSubscriber) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(pub->publish(Text{"nobody listens", {}}));
}

TEST_F(ZmqBusTest, CallsHandlersInSubscriptionOrderUntilReleased) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    const auto sub = subscriber(bus, "fc/state");
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    std::vector<std::string> calls;
    core::Subscription first =
        sub->subscribe([&](const Text&, const MessageInfo&) { calls.emplace_back("first"); });
    const core::Subscription second =
        sub->subscribe([&](const Text&, const MessageInfo&) { calls.emplace_back("second"); });
    EXPECT_TRUE(pub->publish(Text{"1", {}}));
    ASSERT_TRUE(pump_until([&] { return calls.size() >= 2; }));
    EXPECT_EQ(calls, (std::vector<std::string>{"first", "second"}));

    first.reset();
    calls.clear();
    EXPECT_TRUE(pub->publish(Text{"2", {}}));
    ASSERT_TRUE(pump_until([&] { return !calls.empty(); }));
    pump_rounds(4);
    EXPECT_EQ(calls, std::vector<std::string>{"second"});
}

TEST_F(ZmqBusTest, SubscriberMayBeDestroyedInsideHandler) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    const auto pub = publisher(bus, "fc/state");
    auto sub = subscriber(bus, "fc/state");
    ASSERT_TRUE(wait_until_connected(*pub, *sub));

    int first_calls = 0;
    int second_calls = 0;
    core::Subscription first = sub->subscribe([&](const Text&, const MessageInfo&) {
        ++first_calls;
        sub.reset();
    });
    core::Subscription second =
        sub->subscribe([&](const Text&, const MessageInfo&) { ++second_calls; });
    EXPECT_TRUE(pub->publish(Text{"1", {}}));
    ASSERT_TRUE(pump_until([&] { return first_calls > 0; }));
    pump_rounds(4);

    EXPECT_EQ(first_calls, 1);
    EXPECT_EQ(second_calls, 0);  // 구독자가 사라진 뒤에는 남은 handler 를 부르지 않는다
    first.reset();               // 구독자보다 오래 산 Subscription 의 해제도 안전하다
    second.reset();
}

// ------------------------------------------------------------------------------------ 생성 실패

TEST_F(ZmqBusTest, ReturnsNullForNameMissingFromEndpointTable) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    EXPECT_EQ(bus.create_publisher("unknown", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_subscriber("unknown", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_requester("unknown", text_type(), text_type()), nullptr);
    EXPECT_EQ(bus.create_replier("unknown", text_type(), text_type()), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullForEmptyName) {
    ZmqBus bus(context_, poller_, clock_, {{"", endpoint("pub")}}, codecs());
    EXPECT_EQ(bus.create_publisher("", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_subscriber("", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_requester("", text_type(), text_type()), nullptr);
    EXPECT_EQ(bus.create_replier("", text_type(), text_type()), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullForTypeMissingFromCodecTable) {
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/state", endpoint("pub")}, {"follow/command", endpoint("rep")}}, codecs());
    const bus::MessageType unregistered = bus::message_type<Unregistered>("test.Unregistered");

    EXPECT_NE(bus.create_publisher("fc/state", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_publisher("fc/state", unregistered, kDepth), nullptr);
    EXPECT_EQ(bus.create_subscriber("fc/state", unregistered, kDepth), nullptr);
    // 요청과 응답 중 하나만 표에 없어도 만들지 않는다
    EXPECT_EQ(bus.create_requester("follow/command", unregistered, text_type()), nullptr);
    EXPECT_EQ(bus.create_requester("follow/command", text_type(), unregistered), nullptr);
    EXPECT_EQ(bus.create_replier("follow/command", unregistered, text_type()), nullptr);
    EXPECT_EQ(bus.create_replier("follow/command", text_type(), unregistered), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullForCodecWithMissingFunction) {
    ZmqCodecTable table = codecs();
    table.at(text_type().type).decode = nullptr;
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, std::move(table));

    EXPECT_EQ(bus.create_publisher("fc/state", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_subscriber("fc/state", text_type(), kDepth), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullForQueueDepthBelowOne) {
    ZmqBus bus(context_, poller_, clock_, {{"fc/state", endpoint("pub")}}, codecs());
    EXPECT_EQ(bus.create_publisher("fc/state", text_type(), 0), nullptr);
    EXPECT_EQ(bus.create_publisher("fc/state", text_type(), -1), nullptr);
    EXPECT_EQ(bus.create_subscriber("fc/state", text_type(), 0), nullptr);
    EXPECT_EQ(bus.create_subscriber("fc/state", text_type(), -1), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullWhenEndpointCannotBeUsed) {
    ZmqBus bus(context_, poller_, clock_,
               {{"topic", "bogus://nowhere"}, {"rpc", "bogus://nowhere"}}, codecs());
    EXPECT_EQ(bus.create_publisher("topic", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_subscriber("topic", text_type(), kDepth), nullptr);
    EXPECT_EQ(bus.create_requester("rpc", text_type(), text_type()), nullptr);
    EXPECT_EQ(bus.create_replier("rpc", text_type(), text_type()), nullptr);
}

TEST_F(ZmqBusTest, ReturnsNullWhenEndpointIsAlreadyBound) {
    ZmqBus first(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    ZmqBus second(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(first, "follow/command");
    ASSERT_NE(rep, nullptr);
    EXPECT_EQ(replier(second, "follow/command"), nullptr);
}

// ------------------------------------------------------------------------------------ 요청-응답

// "<접두사>:<요청의 text>" 로 응답하는 handler
bus::Replier<Text, Text>::RequestHandler echo_with(const std::string& prefix) {
    // prefix 를 const 가 아닌 복사본으로 잡는다 (const 멤버가 있으면 람다를 이동할 수 없다)
    return [reply_prefix = prefix](const Text& request) {
        return Text{reply_prefix + ":" + request.text, {}};
    };
}

// 응답을 기록한다. timeout 은 "<timeout>" 으로 기록한다
bus::Requester<Text, Text>::ReplyHandler record_into(std::vector<std::string>& replies) {
    return [&replies](const std::optional<Text>& reply) {
        replies.push_back(reply ? reply->text : "<timeout>");
    };
}

TEST_F(ZmqBusTest, DeliversReplyToRequester) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    ASSERT_NE(rep, nullptr);
    ASSERT_NE(req, nullptr);
    EXPECT_EQ(rep->name(), "follow/command");
    EXPECT_EQ(req->name(), "follow/command");

    std::vector<std::string> requests;
    const core::Subscription serving = rep->serve([&](const Text& request) {
        requests.push_back(request.text);
        return Text{"accepted", {}};
    });
    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"start", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));

    EXPECT_EQ(requests, std::vector<std::string>{"start"});
    EXPECT_EQ(replies, std::vector<std::string>{"accepted"});
}

// 요청과 응답의 전송 바이트: [0] 이름, [1] 본문. ZmqBus 를 거치지 않는 REQ 소켓으로 확인한다
TEST_F(ZmqBusTest, RequestAndReplyAreNameAndBodyFrames) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, plain_codecs());
    const auto rep = replier(bus, "follow/command");
    ASSERT_NE(rep, nullptr);
    const core::Subscription serving = rep->serve(echo_with("re"));
    zmq::socket_t raw(context_, zmq::socket_type::req);
    raw.set(zmq::sockopt::rcvtimeo, 0);
    raw.connect(endpoint("rep"));

    raw.send(zmq::str_buffer("follow/command"), zmq::send_flags::sndmore);
    raw.send(zmq::str_buffer("start"), zmq::send_flags::none);
    // 응답 하나의 프레임들을 읽는다. 다 읽은 뒤에는 다시 읽지 않는다 (REQ 소켓은 응답을 받은
    // 뒤에 다시 읽으면 오류가 난다)
    std::vector<std::string> frames;
    bool complete = false;
    ASSERT_TRUE(pump_until([&] {
        zmq::message_t frame;
        while (!complete && raw.recv(frame).has_value()) {
            frames.push_back(frame.to_string());
            complete = !frame.more();
        }
        return complete;
    }));

    EXPECT_EQ(frames, (std::vector<std::string>{"follow/command", "re:start"}));
}

TEST_F(ZmqBusTest, RepliesWithEmptyBodyToRequestThatCannotBeDecoded) {
    // 요청하는 bus 는 번호 없는 본문을 보내고, 응답하는 bus 의 decode 는 그 본문을 거부한다
    ZmqBus client(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, plain_codecs());
    ZmqBus server(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(server, "follow/command");
    const auto req = requester(client, "follow/command");
    int served = 0;
    const core::Subscription serving = rep->serve([&](const Text&) {
        ++served;
        return Text{"accepted", {}};
    });

    std::vector<std::string> replies;
    const core::Subscription first =
        req->request(Text{"broken", {}}, milliseconds{1000}, record_into(replies));
    const core::Subscription second =
        req->request(Text{"0|valid", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return replies.size() == 2; }));

    EXPECT_EQ(served, 1);  // 형식이 맞지 않는 요청에는 handler 를 부르지 않는다
    // 응답하는 쪽은 "0|accepted" 를 보내고, 요청하는 쪽의 변환 표는 본문을 그대로 text 로 읽는다
    EXPECT_EQ(replies, (std::vector<std::string>{"", "0|accepted"}));
}

TEST_F(ZmqBusTest, ReportsNoReplyWhenReplyCannotBeDecoded) {
    // 응답하는 bus 는 번호 없는 본문을 보내고, 요청하는 bus 의 decode 는 그 본문을 거부한다
    ZmqBus client(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    ZmqBus server(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, plain_codecs());
    const auto rep = replier(server, "follow/command");
    const auto req = requester(client, "follow/command");
    const core::Subscription serving = rep->serve(echo_with("re"));

    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"1", {}}, milliseconds{1000}, record_into(replies));
    // 시각을 옮기지 않으므로 timeout 은 지나지 않는다. 응답이 도착한 때에 한 번 불린다
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));
    pump_rounds(4);

    EXPECT_EQ(replies, std::vector<std::string>{"<timeout>"});
}

TEST_F(ZmqBusTest, RequestThatCannotBeEncodedEndsAtTimeoutWithoutBlockingOthers) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    std::vector<std::string> requests;
    const core::Subscription serving = rep->serve([&](const Text& request) {
        requests.push_back(request.text);
        return Text{"re:" + request.text, {}};
    });

    std::vector<std::string> failed_replies;
    std::vector<std::string> replies;
    const core::Subscription unencodable =
        req->request(Text{kUnencodable, {}}, milliseconds{50}, record_into(failed_replies));
    // 요청에는 바이너리를 실을 수 없다
    const core::Subscription with_binary =
        req->request(Text{"binary", {1}}, milliseconds{50}, record_into(failed_replies));
    const core::Subscription next =
        req->request(Text{"next", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));

    EXPECT_EQ(replies, std::vector<std::string>{"re:next"});
    EXPECT_EQ(requests, std::vector<std::string>{"next"});
    EXPECT_TRUE(failed_replies.empty());

    clock_.advance_ns(50 * kNsPerMs);
    pump_rounds(2);
    EXPECT_EQ(failed_replies, (std::vector<std::string>{"<timeout>", "<timeout>"}));
}

TEST_F(ZmqBusTest, RepliesWithEmptyBodyWhenReplyCannotBeEncoded) {
    ZmqBus client(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, plain_codecs());
    ZmqBus server(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(server, "follow/command");
    const auto req = requester(client, "follow/command");
    const core::Subscription serving =
        rep->serve([](const Text&) { return Text{kUnencodable, {}}; });

    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"0|valid", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));

    EXPECT_EQ(replies, std::vector<std::string>{""});
}

TEST_F(ZmqBusTest, AnswersQueuedRequestsInOrder) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    const core::Subscription serving = rep->serve(echo_with("re"));

    std::vector<std::string> replies;
    const core::Subscription first =
        req->request(Text{"1", {}}, milliseconds{1000}, record_into(replies));
    const core::Subscription second =
        req->request(Text{"2", {}}, milliseconds{1000}, record_into(replies));
    const core::Subscription third =
        req->request(Text{"3", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return replies.size() == 3; }));

    EXPECT_EQ(replies, (std::vector<std::string>{"re:1", "re:2", "re:3"}));
}

TEST_F(ZmqBusTest, ReportsTimeoutOnceWhenNoReplyArrives) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto req = requester(bus, "follow/command");  // 응답하는 쪽이 없다

    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"1", {}}, milliseconds{50}, record_into(replies));
    pump_rounds(4);
    EXPECT_TRUE(replies.empty());

    clock_.advance_ns(49 * kNsPerMs);
    pump_rounds(2);
    EXPECT_TRUE(replies.empty());

    clock_.advance_ns(1 * kNsPerMs);
    pump_rounds(2);
    EXPECT_EQ(replies, std::vector<std::string>{"<timeout>"});

    clock_.advance_ns(100 * kNsPerMs);
    pump_rounds(2);
    EXPECT_EQ(replies, std::vector<std::string>{"<timeout>"});
}

TEST_F(ZmqBusTest, DoesNotDeliverReplyThatArrivesAfterTimeout) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");  // 아직 serve() 하지 않는다
    const auto req = requester(bus, "follow/command");

    std::vector<std::string> replies;
    const core::Subscription first =
        req->request(Text{"old", {}}, milliseconds{50}, record_into(replies));
    pump_rounds(4);
    clock_.advance_ns(50 * kNsPerMs);
    pump_rounds(2);
    ASSERT_EQ(replies, std::vector<std::string>{"<timeout>"});

    // 이제 응답하는 쪽이 밀린 요청에 응답한다. 요청한 쪽은 이미 끝낸 요청이므로 전달하지 않는다
    const core::Subscription serving = rep->serve(echo_with("re"));
    pump_rounds(20);
    EXPECT_EQ(replies, std::vector<std::string>{"<timeout>"});

    // 다음 요청은 자기 응답을 받는다 (앞 요청의 응답과 섞이지 않는다)
    const core::Subscription second =
        req->request(Text{"new", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return replies.size() == 2; }));
    EXPECT_EQ(replies, (std::vector<std::string>{"<timeout>", "re:new"}));
}

TEST_F(ZmqBusTest, QueuedRequestTimesOutWhileEarlierRequestWaits) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto req = requester(bus, "follow/command");  // 응답하는 쪽이 없다

    std::vector<std::string> first_replies;
    std::vector<std::string> second_replies;
    const core::Subscription first =
        req->request(Text{"1", {}}, milliseconds{100}, record_into(first_replies));
    const core::Subscription second =
        req->request(Text{"2", {}}, milliseconds{30}, record_into(second_replies));

    clock_.advance_ns(30 * kNsPerMs);
    pump_rounds(2);
    EXPECT_TRUE(first_replies.empty());
    EXPECT_EQ(second_replies, std::vector<std::string>{"<timeout>"});

    clock_.advance_ns(70 * kNsPerMs);
    pump_rounds(2);
    EXPECT_EQ(first_replies, std::vector<std::string>{"<timeout>"});
}

TEST_F(ZmqBusTest, DoesNotCallReplyHandlerAfterRequestIsReleased) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    const core::Subscription serving = rep->serve(echo_with("re"));

    std::vector<std::string> replies;
    core::Subscription released =
        req->request(Text{"released", {}}, milliseconds{50}, record_into(replies));
    released.reset();
    pump_rounds(20);
    clock_.advance_ns(100 * kNsPerMs);
    pump_rounds(2);
    EXPECT_TRUE(replies.empty());  // 응답도 timeout 도 전달하지 않는다

    const core::Subscription next =
        req->request(Text{"next", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));
    EXPECT_EQ(replies, std::vector<std::string>{"re:next"});
}

TEST_F(ZmqBusTest, ServeAgainReplacesHandler) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    core::Subscription old_serving = rep->serve(echo_with("old"));
    const core::Subscription new_serving = rep->serve(echo_with("new"));
    old_serving.reset();  // 대체된 등록의 해제는 새 등록에 영향을 주지 않는다

    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"1", {}}, milliseconds{1000}, record_into(replies));
    ASSERT_TRUE(pump_until([&] { return !replies.empty(); }));

    EXPECT_EQ(replies, std::vector<std::string>{"new:1"});
}

TEST_F(ZmqBusTest, DoesNotReplyAfterServeIsReleased) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    const auto req = requester(bus, "follow/command");
    core::Subscription serving = rep->serve(echo_with("re"));
    serving.reset();

    std::vector<std::string> replies;
    const core::Subscription pending =
        req->request(Text{"1", {}}, milliseconds{50}, record_into(replies));
    pump_rounds(20);
    EXPECT_TRUE(replies.empty());

    clock_.advance_ns(50 * kNsPerMs);
    pump_rounds(2);
    EXPECT_EQ(replies, std::vector<std::string>{"<timeout>"});
}

TEST_F(ZmqBusTest, RequesterMayBeDestroyedInsideReplyHandler) {
    ZmqBus bus(context_, poller_, clock_, {{"follow/command", endpoint("rep")}}, codecs());
    const auto rep = replier(bus, "follow/command");
    auto req = requester(bus, "follow/command");
    const core::Subscription serving = rep->serve(echo_with("re"));

    int first_calls = 0;
    int second_calls = 0;
    core::Subscription first =
        req->request(Text{"1", {}}, milliseconds{1000}, [&](const std::optional<Text>&) {
            ++first_calls;
            req.reset();
        });
    core::Subscription second = req->request(Text{"2", {}}, milliseconds{1000},
                                             [&](const std::optional<Text>&) { ++second_calls; });
    ASSERT_TRUE(pump_until([&] { return first_calls > 0; }));
    clock_.advance_ns(2000 * kNsPerMs);
    pump_rounds(4);

    EXPECT_EQ(first_calls, 1);
    EXPECT_EQ(second_calls, 0);  // 요청하는 쪽이 사라지면 남은 요청은 응답도 timeout 도 받지 않는다
    first.reset();
    second.reset();
}

// 래퍼(argus/bus/typed.hpp)는 빈 handler 를 bus 에 넘기지 않으므로 인터페이스를 직접 부른다
TEST_F(ZmqBusTest, RejectsEmptyHandlers) {
    ZmqBus bus(context_, poller_, clock_,
               {{"fc/state", endpoint("pub")}, {"follow/command", endpoint("rep")}}, codecs());
    const auto sub = bus.create_subscriber("fc/state", text_type(), kDepth);
    const auto rep = bus.create_replier("follow/command", text_type(), text_type());
    const auto req = bus.create_requester("follow/command", text_type(), text_type());
    ASSERT_NE(sub, nullptr);
    ASSERT_NE(rep, nullptr);
    ASSERT_NE(req, nullptr);
    const Text request{"1", {}};

    EXPECT_FALSE(sub->subscribe(nullptr).active());
    EXPECT_FALSE(rep->serve(nullptr).active());
    EXPECT_FALSE(req->request(&request, milliseconds{10}, nullptr).active());
}

}  // namespace
}  // namespace argus::bus_zmq
