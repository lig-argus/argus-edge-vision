#include "argus/bus_zmq/zmq_bus.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

#include "argus/bus/message_type.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/core/subscription.hpp"
#include "detail.hpp"

namespace argus::bus_zmq {
namespace {

using detail::kNsPerMs;
using detail::log_error;

// 구독자가 메시지를 읽지 못한 동안 온 메시지는 운영체제의 소켓 버퍼에 쌓인다. libzmq 는 자기 큐에
// 최대 ZMQ_RCVHWM 개까지만 두고, 읽는 쪽이 그 절반을 읽을 때마다 다른 스레드에서 다시 채운다.
// 그래서 운영체제의 버퍼에 메시지가 남아 있어도 큐가 잠시 비어 있을 수 있다. 큐가 비어 있으면 다음
// 메시지가 채워질 때까지 kDrainWait 동안 기다린다. 읽기를 시작한 시점부터 kDrainBudget 이 지나면 더
// 기다리지 않고 읽기를 끝낸다 (발행 속도가 읽는 속도보다 빠르면 읽기가 끝나지 않으므로)
constexpr std::chrono::milliseconds kDrainWait{1};
constexpr std::chrono::milliseconds kDrainBudget{20};

// 한 번에 읽은 메시지가 이 수 이상이면 구독자가 메시지를 제때 읽지 못한 것으로 본다
constexpr std::size_t kBacklogMessageCount = 2;

// 요청과 응답은 발행 번호가 없다. 변환 함수의 sequence 인자에 이 값을 넘긴다
constexpr std::uint64_t kNoSequence = 0;

zmq::const_buffer as_buffer(std::string_view text) { return {text.data(), text.size()}; }
zmq::const_buffer as_buffer(const Bytes& bytes) { return {bytes.data(), bytes.size()}; }

Bytes to_bytes(const zmq::message_t& frame) {
    const auto* begin = static_cast<const std::uint8_t*>(frame.data());
    return {begin, begin + frame.size()};
}

// 메시지 하나(프레임 여러 개)를 기다리지 않고 읽는다. 읽을 메시지가 없거나 오류이면 false
bool receive_frames(zmq::socket_t& socket, std::vector<zmq::message_t>& frames) {
    frames.clear();
    try {
        for (;;) {
            zmq::message_t frame;
            if (!socket.recv(frame, zmq::recv_flags::dontwait).has_value()) {
                return false;
            }
            const bool more = frame.more();
            frames.push_back(std::move(frame));
            if (!more) {
                return true;
            }
        }
    } catch (const zmq::error_t& error) {
        log_error(std::string("수신 실패: ") + error.what());
        return false;
    }
}

// multipart 구성 (docs/spec/process_interfaces.md 3장): [0] 이름, [1] 본문, 필요하면 [2] 바이너리.
// 이름은 토픽 이름 또는 요청-응답 이름이다. 이 구성을 아는 코드는 send_message() 와 to_wire()
// 둘뿐이다
constexpr std::size_t kNameFrame = 0;
constexpr std::size_t kBodyFrame = 1;
constexpr std::size_t kBinaryFrame = 2;
constexpr std::size_t kFrameCountWithoutBinary = 2;
constexpr std::size_t kFrameCountWithBinary = 3;

// 메시지 하나를 기다리지 않고 보낸다. binary 가 비어 있으면 바이너리 프레임을 보내지 않는다.
// 큐에 넣지 못했거나 오류이면 false. 기다리지 않는 것은 소켓의 ZMQ_SNDTIMEO 0 으로 정한다
// (disable_send_wait). 호출마다 dontwait 를 주면 sndmore 와 합친 값이 zmq::send_flags 에 정의되지
// 않은 값이 되어 정적 분석이 오류로 본다
bool send_message(zmq::socket_t& socket, std::string_view name, const Bytes& body,
                  const Bytes& binary) {
    std::vector<zmq::const_buffer> frames(kFrameCountWithoutBinary);
    frames[kNameFrame] = as_buffer(name);
    frames[kBodyFrame] = as_buffer(body);
    if (!binary.empty()) {
        frames.push_back(as_buffer(binary));
    }
    try {
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const bool last = i + 1 == frames.size();
            const zmq::send_flags flags = last ? zmq::send_flags::none : zmq::send_flags::sndmore;
            if (!socket.send(frames[i], flags).has_value()) {
                return false;
            }
        }
        return true;
    } catch (const zmq::error_t& error) {
        log_error(std::string("송신 실패: ") + error.what());
        return false;
    }
}

// 보내는 소켓(PUB, REQ, REP)을 만들 때 부른다. 큐에 넣지 못하는 경우 send 가 기다리지 않고 바로
// 실패를 반환한다
void disable_send_wait(zmq::socket_t& socket) { socket.set(zmq::sockopt::sndtimeo, 0); }

// 메시지 하나의 전송 바이트 (이름 프레임 제외)
struct WireMessage {
    Bytes body;    // 직렬화된 본문
    Bytes binary;  // 선택. 큰 바이너리(예: gray8 픽셀). 비어 있으면 없음
};

// 받은 프레임들에서 본문과 바이너리를 꺼낸다. 프레임 수가 규약과 다르거나 이름이 expected_name 과
// 다르면 nullopt
std::optional<WireMessage> to_wire(const std::vector<zmq::message_t>& frames,
                                   std::string_view expected_name) {
    const bool has_binary = frames.size() == kFrameCountWithBinary;
    if (frames.size() != kFrameCountWithoutBinary && !has_binary) {
        return std::nullopt;
    }
    if (frames[kNameFrame].to_string_view() != expected_name) {
        return std::nullopt;
    }
    WireMessage message;
    message.body = to_bytes(frames[kBodyFrame]);
    if (has_binary) {
        message.binary = to_bytes(frames[kBinaryFrame]);
    }
    return message;
}

// ---------------------------------------------------------------------------------------------
// 토픽 발행. 엔드포인트마다 PUB 소켓 하나를 그 엔드포인트의 발행자들이 함께 쓴다

struct PubSocket {
    PubSocket(zmq::context_t& context, std::string endpoint_text, int depth)
        : socket(context, zmq::socket_type::pub),
          endpoint(std::move(endpoint_text)),
          queue_depth(depth) {
        socket.set(zmq::sockopt::linger, 0);  // 닫을 때 아직 보내지 못한 메시지는 버린다
        disable_send_wait(socket);
        socket.set(zmq::sockopt::sndhwm, queue_depth);
        socket.bind(endpoint);
    }

    // 큐 길이는 bind 할 때의 값이 그 뒤 연결에 적용되므로 다시 bind 한다
    void grow_queue_depth(int depth) {
        socket.unbind(endpoint);
        socket.set(zmq::sockopt::sndhwm, depth);
        socket.bind(endpoint);
        queue_depth = depth;
    }

    zmq::socket_t socket;
    std::string endpoint;
    int queue_depth;
};

class ZmqTopicPublisher : public bus::ITopicPublisher {
public:
    ZmqTopicPublisher(std::shared_ptr<PubSocket> socket, std::string_view topic, ZmqCodec codec)
        : socket_(std::move(socket)), topic_(topic), codec_(std::move(codec)) {}

    std::string_view topic() const override { return topic_; }

    bool publish(const void* message) override {
        // 보내지 못한 발행도 번호를 쓴다. 받는 쪽이 그 메시지를 유실로 세게 된다
        const std::uint64_t sequence = ++sequence_;
        // 발행마다 메모리를 다시 할당하지 않도록 버퍼를 멤버로 두고 비워서 쓴다
        body_.clear();
        binary_.clear();
        if (!codec_.encode(message, sequence, body_, binary_)) {
            log_error("publish(" + topic_ + "): 값을 전송 형식으로 바꾸지 못함");
            return false;
        }
        return send_message(socket_->socket, topic_, body_, binary_);
    }

private:
    std::shared_ptr<PubSocket> socket_;
    std::string topic_;
    ZmqCodec codec_;
    std::uint64_t sequence_ = 0;  // 마지막으로 쓴 발행 번호
    Bytes body_;
    Bytes binary_;
};

// ---------------------------------------------------------------------------------------------
// 구독자, 요청하는 쪽, 응답하는 쪽은 상태를 공유 객체(Core)에 둔다. poller 와 Subscription 은 Core
// 를 소유하지 않고 weak_ptr 로 참조한다. Core 가 이미 사라졌으면 아무 일도 하지 않으므로, handler
// 안에서 자기 객체를 없애거나 Subscription 이 객체보다 오래 살아도 안전하다. 바깥 클래스의 소멸자가
// close() 를 불러 등록을 풀고 소켓을 닫는다

class SubscriberCore : public std::enable_shared_from_this<SubscriberCore> {
public:
    using Handler = bus::ITopicSubscriber::Handler;

    SubscriberCore(zmq::context_t& context, ZmqPoller& poller, const core::IClock& clock,
                   const std::string& endpoint, std::string_view topic,
                   const bus::MessageType& type, ZmqCodec codec, int queue_depth)
        : poller_(poller),
          clock_(clock),
          topic_(topic),
          type_(type),
          codec_(std::move(codec)),
          queue_depth_(static_cast<std::size_t>(queue_depth)),
          socket_(context, zmq::socket_type::sub) {
        socket_.set(zmq::sockopt::linger, 0);
        socket_.set(zmq::sockopt::rcvhwm, queue_depth);
        socket_.set(zmq::sockopt::subscribe, topic_);
        socket_.connect(endpoint);
    }

    void start() {
        std::weak_ptr<SubscriberCore> weak = weak_from_this();
        poll_ = poller_.add_socket(socket_, [weak] {
            if (const auto self = weak.lock()) {
                self->on_readable();
            }
        });
    }

    void close() {
        closed_ = true;
        handlers_.clear();
        poll_.reset();
        socket_.close();
    }

    std::string_view topic() const { return topic_; }

    core::Subscription subscribe(Handler handler) {
        if (!handler) {
            log_error("subscribe(" + topic_ + "): 빈 handler");
            return {};
        }
        const std::uint64_t id = next_id_++;
        handlers_.emplace(id, std::move(handler));
        std::weak_ptr<SubscriberCore> weak = weak_from_this();
        return core::Subscription([weak, id] {
            if (const auto self = weak.lock()) {
                self->handlers_.erase(id);
            }
        });
    }

private:
    void on_readable() {
        for (const WireMessage& message : read_latest()) {
            if (closed_) {
                return;
            }
            deliver(message);
        }
    }

    // 쌓인 메시지를 모두 읽고 최근 queue_depth_ 개를 받은 순서대로 반환한다. 작은 메시지는
    // ZMQ_RCVHWM 을 넘겨도 운영체제의 소켓 버퍼에 쌓이므로, 끝까지 읽어야 오래된 메시지를 폐기할
    // 수 있다. ZMQ_CONFLATE 는 multipart 메시지를 지원하지 않아 사용하지 않는다.
    // 읽은 메시지가 kBacklogMessageCount 이상이면 큐가 비어 있어도 kDrainWait 동안 기다려 이어서
    // 읽는다 (이유는 kDrainWait 의 설명). 메시지가 하나만 와 있었으면 기다리지 않는다
    std::deque<WireMessage> read_latest() {
        std::deque<WireMessage> latest;
        std::size_t read_count = 0;
        const std::int64_t budget_end_ns =
            clock_.now_monotonic_ns() + kDrainBudget.count() * kNsPerMs;
        for (;;) {
            read_count += read_available(latest);
            const bool fell_behind = read_count >= kBacklogMessageCount;
            const bool budget_left = clock_.now_monotonic_ns() < budget_end_ns;
            if (!fell_behind || !budget_left || !wait_readable()) {
                return latest;
            }
        }
    }

    // libzmq 의 큐에 있는 메시지를 모두 읽어 latest 에 최근 queue_depth_ 개만 남긴다. 읽은 메시지
    // 수를 반환한다
    std::size_t read_available(std::deque<WireMessage>& latest) {
        std::vector<zmq::message_t> frames;
        std::size_t read_count = 0;
        while (receive_frames(socket_, frames)) {
            ++read_count;
            // ZeroMQ 의 구독 필터는 이름의 앞부분만 비교하므로 이름 전체가 같은 메시지만 받는다
            std::optional<WireMessage> received = to_wire(frames, topic_);
            if (!received) {
                continue;
            }
            if (latest.size() == queue_depth_) {
                latest.pop_front();
            }
            latest.push_back(std::move(*received));
        }
        return read_count;
    }

    // kDrainWait 안에 읽을 메시지가 생기면 true
    bool wait_readable() {
        zmq::pollitem_t item{socket_.handle(), 0, ZMQ_POLLIN, 0};
        try {
            return zmq::poll(&item, 1, kDrainWait) > 0;
        } catch (const zmq::error_t& error) {
            log_error(std::string("수신 대기 실패: ") + error.what());
            return false;
        }
    }

    // 전송 바이트를 값으로 바꿔 handler 들에 전달한다. 폐기되지 않고 남은 메시지만 바꾼다
    void deliver(const WireMessage& message) {
        if (handlers_.empty()) {
            return;  // 받을 handler 가 없으면 값으로 바꾸지 않는다
        }
        const std::shared_ptr<void> value = type_.create();
        bus::MessageInfo info;
        if (!codec_.decode(message.body, message.binary, value.get(), info.sequence)) {
            log_error("subscriber(" + topic_ + "): 형식이 맞지 않는 메시지 폐기");
            return;
        }
        std::vector<std::uint64_t> ids;
        ids.reserve(handlers_.size());
        for (const auto& [id, handler] : handlers_) {
            ids.push_back(id);
        }
        for (const std::uint64_t id : ids) {
            // 앞 handler 가 이 등록을 해제했거나 구독자를 없앴을 수 있다
            const auto found = handlers_.find(id);
            if (closed_ || found == handlers_.end()) {
                continue;
            }
            const Handler handler = found->second;
            handler(value.get(), info);
        }
    }

    ZmqPoller& poller_;
    const core::IClock& clock_;
    std::string topic_;
    bus::MessageType type_;
    ZmqCodec codec_;
    std::size_t queue_depth_;
    zmq::socket_t socket_;
    std::map<std::uint64_t, Handler> handlers_;  // 등록한 순서
    std::uint64_t next_id_ = 1;
    bool closed_ = false;
    core::Subscription poll_;
};

class ZmqTopicSubscriber : public bus::ITopicSubscriber {
public:
    explicit ZmqTopicSubscriber(std::shared_ptr<SubscriberCore> core) : core_(std::move(core)) {
        core_->start();
    }
    ~ZmqTopicSubscriber() override { core_->close(); }
    ZmqTopicSubscriber(const ZmqTopicSubscriber&) = delete;
    ZmqTopicSubscriber& operator=(const ZmqTopicSubscriber&) = delete;
    ZmqTopicSubscriber(ZmqTopicSubscriber&&) = delete;
    ZmqTopicSubscriber& operator=(ZmqTopicSubscriber&&) = delete;

    std::string_view topic() const override { return core_->topic(); }
    core::Subscription subscribe(Handler handler) override {
        return core_->subscribe(std::move(handler));
    }

private:
    std::shared_ptr<SubscriberCore> core_;
};

// ---------------------------------------------------------------------------------------------
// 요청하는 쪽. REQ 소켓은 응답을 받기 전에 다음 요청을 보낼 수 없으므로 요청을 큐에 두고 하나씩
// 보낸다. ZMQ_REQ_RELAXED 로 응답 없이 다음 요청을 보낼 수 있게 하고, ZMQ_REQ_CORRELATE 로 앞
// 요청의 늦은 응답을 libzmq 가 버리게 한다

class RequesterCore : public std::enable_shared_from_this<RequesterCore> {
public:
    using ReplyHandler = bus::IRequester::ReplyHandler;

    RequesterCore(zmq::context_t& context, ZmqPoller& poller, const core::IClock& clock,
                  const std::string& endpoint, std::string_view name, ZmqCodec request_codec,
                  const bus::MessageType& reply_type, ZmqCodec reply_codec)
        : poller_(poller),
          clock_(clock),
          name_(name),
          request_codec_(std::move(request_codec)),
          reply_type_(reply_type),
          reply_codec_(std::move(reply_codec)),
          socket_(context, zmq::socket_type::req) {
        socket_.set(zmq::sockopt::linger, 0);
        disable_send_wait(socket_);
        socket_.set(zmq::sockopt::req_relaxed, 1);
        socket_.set(zmq::sockopt::req_correlate, 1);
        socket_.connect(endpoint);
    }

    void start() {
        std::weak_ptr<RequesterCore> weak = weak_from_this();
        poll_ = poller_.add_socket(socket_, [weak] {
            if (const auto self = weak.lock()) {
                self->on_readable();
            }
        });
    }

    void close() {
        closed_ = true;
        queue_.clear();  // 타이머 등록도 함께 풀린다
        unsendable_.clear();
        poll_.reset();
        socket_.close();
    }

    std::string_view name() const { return name_; }

    core::Subscription request(const void* request, std::chrono::milliseconds timeout,
                               ReplyHandler on_reply) {
        if (!on_reply) {
            log_error("request(" + name_ + "): 빈 on_reply");
            return {};
        }
        const std::uint64_t id = next_id_++;
        const std::int64_t deadline_ns =
            clock_.now_monotonic_ns() + std::max<std::int64_t>(timeout.count(), 0) * kNsPerMs;
        std::weak_ptr<RequesterCore> weak = weak_from_this();

        // request 는 이 함수가 끝나면 유효하지 않으므로 지금 전송 바이트로 바꿔 보관한다
        Pending pending;
        pending.id = id;
        Bytes binary;
        const bool encoded =
            request_codec_.encode(request, kNoSequence, pending.request, binary) && binary.empty();
        pending.on_reply = std::move(on_reply);
        pending.timer = poller_.add_timer_at(deadline_ns, [weak, id] {
            if (const auto self = weak.lock()) {
                self->finish(id, true);
            }
        });
        if (encoded) {
            queue_.push_back(std::move(pending));
            send_next();
        } else {
            // 보낼 수 없는 요청은 큐에 넣지 않는다 (뒤 요청을 막지 않게). timeout 에 끝낸다
            log_error("request(" + name_ + "): 요청을 전송 형식으로 바꾸지 못함");
            unsendable_.push_back(std::move(pending));
        }

        return core::Subscription([weak, id] {
            if (const auto self = weak.lock()) {
                self->finish(id, false);
            }
        });
    }

private:
    struct Pending {
        std::uint64_t id = 0;
        Bytes request;  // 전송 형식으로 바꾼 요청 본문
        ReplyHandler on_reply;
        core::Subscription timer;
    };

    // 보낸 요청이 없으면 큐의 맨 앞 요청을 보낸다
    void send_next() {
        if (closed_ || in_flight_ || queue_.empty()) {
            return;
        }
        in_flight_ = send_message(socket_, name_, queue_.front().request, {});
    }

    void on_readable() {
        std::vector<zmq::message_t> frames;
        if (!receive_frames(socket_, frames)) {
            return;
        }
        // 기다리는 요청이 없는데 온 응답(취소했거나 timeout 이 지난 요청의 응답)과 프레임 구성이
        // 다른 응답은 버린다. 응답에는 바이너리가 없다
        const std::optional<WireMessage> wire = to_wire(frames, name_);
        if (!in_flight_ || !wire || !wire->binary.empty()) {
            return;
        }
        Pending done = std::move(queue_.front());
        queue_.pop_front();
        in_flight_ = false;
        done.timer.reset();

        // 값으로 바꾸지 못한 응답은 응답이 오지 않은 것과 같이 다룬다
        const std::shared_ptr<void> reply = reply_type_.create();
        std::uint64_t sequence = kNoSequence;
        const bool decoded = reply_codec_.decode(wire->body, wire->binary, reply.get(), sequence);
        if (!decoded) {
            log_error("requester(" + name_ + "): 형식이 맞지 않는 응답 폐기");
        }
        done.on_reply(decoded ? reply.get() : nullptr);
        send_next();
    }

    // 요청 하나를 응답 없이 끝낸다. timed_out 이면 on_reply(nullptr) 를 부르고, 아니면(요청한 쪽이
    // Subscription 을 해제함) 부르지 않는다
    void finish(std::uint64_t id, bool timed_out) {
        const auto has_id = [id](const Pending& pending) { return pending.id == id; };
        Pending done;
        if (const auto found = std::find_if(queue_.begin(), queue_.end(), has_id);
            found != queue_.end()) {
            if (found == queue_.begin() && in_flight_) {
                in_flight_ = false;
            }
            done = std::move(*found);
            queue_.erase(found);
        } else if (const auto unsent = std::find_if(unsendable_.begin(), unsendable_.end(), has_id);
                   unsent != unsendable_.end()) {
            done = std::move(*unsent);
            unsendable_.erase(unsent);
        } else {
            return;
        }
        if (timed_out) {
            done.on_reply(nullptr);
        }
        send_next();
    }

    ZmqPoller& poller_;
    const core::IClock& clock_;
    std::string name_;
    ZmqCodec request_codec_;
    bus::MessageType reply_type_;
    ZmqCodec reply_codec_;
    zmq::socket_t socket_;
    std::deque<Pending> queue_;        // 보낸 순서. in_flight_ 이면 맨 앞이 응답을 기다리는 요청
    std::vector<Pending> unsendable_;  // 전송 형식으로 바꾸지 못한 요청. timeout 을 기다린다
    bool in_flight_ = false;
    std::uint64_t next_id_ = 1;
    bool closed_ = false;
    core::Subscription poll_;
};

class ZmqRequester : public bus::IRequester {
public:
    explicit ZmqRequester(std::shared_ptr<RequesterCore> core) : core_(std::move(core)) {
        core_->start();
    }
    ~ZmqRequester() override { core_->close(); }
    ZmqRequester(const ZmqRequester&) = delete;
    ZmqRequester& operator=(const ZmqRequester&) = delete;
    ZmqRequester(ZmqRequester&&) = delete;
    ZmqRequester& operator=(ZmqRequester&&) = delete;

    std::string_view name() const override { return core_->name(); }
    core::Subscription request(const void* request, std::chrono::milliseconds timeout,
                               ReplyHandler on_reply) override {
        return core_->request(request, timeout, std::move(on_reply));
    }

private:
    std::shared_ptr<RequesterCore> core_;
};

// ---------------------------------------------------------------------------------------------
// 응답하는 쪽. REP 소켓은 요청을 읽으면 반드시 응답해야 다음 요청을 읽을 수 있다. 그래서 handler 가
// 있을 때만 poller 에 등록해 요청을 읽는다

class ReplierCore : public std::enable_shared_from_this<ReplierCore> {
public:
    using RequestHandler = bus::IReplier::RequestHandler;

    ReplierCore(zmq::context_t& context, ZmqPoller& poller, const std::string& endpoint,
                std::string_view name, const bus::MessageType& request_type, ZmqCodec request_codec,
                const bus::MessageType& reply_type, ZmqCodec reply_codec)
        : poller_(poller),
          name_(name),
          request_type_(request_type),
          request_codec_(std::move(request_codec)),
          reply_type_(reply_type),
          reply_codec_(std::move(reply_codec)),
          socket_(context, zmq::socket_type::rep) {
        socket_.set(zmq::sockopt::linger, 0);
        disable_send_wait(socket_);
        socket_.bind(endpoint);
    }

    void close() {
        closed_ = true;
        handler_ = nullptr;
        poll_.reset();
        socket_.close();
    }

    std::string_view name() const { return name_; }

    core::Subscription serve(RequestHandler handler) {
        if (!handler) {
            log_error("serve(" + name_ + "): 빈 handler");
            return {};
        }
        handler_ = std::move(handler);
        const std::uint64_t registration = ++registration_;
        std::weak_ptr<ReplierCore> weak = weak_from_this();
        if (!poll_.active()) {
            poll_ = poller_.add_socket(socket_, [weak] {
                if (const auto self = weak.lock()) {
                    self->on_readable();
                }
            });
        }
        return core::Subscription([weak, registration] {
            const auto self = weak.lock();
            // 다시 등록해 대체된 뒤에 온 해제는 새 등록을 건드리지 않는다
            if (self && self->registration_ == registration) {
                self->handler_ = nullptr;
                self->poll_.reset();
            }
        });
    }

private:
    void on_readable() {
        std::vector<zmq::message_t> frames;
        if (!receive_frames(socket_, frames)) {
            return;
        }
        const Bytes reply = make_reply(frames);
        if (closed_) {
            return;
        }
        send_message(socket_, name_, reply, {});
    }

    // 요청을 값으로 바꿔 handler 를 부르고, handler 가 채운 응답을 전송 형식으로 바꿔 반환한다.
    // 그렇게 하지 못한 경우에는 이유를 남기고 빈 본문을 반환한다 (REP 소켓이 다음 요청을 읽으려면
    // 응답을 보내야 한다)
    Bytes make_reply(const std::vector<zmq::message_t>& frames) {
        // 요청에는 바이너리가 없다
        const std::optional<WireMessage> wire = to_wire(frames, name_);
        if (!wire || !wire->binary.empty() || !handler_) {
            // 이 엔드포인트에 다른 이름이나 다른 프레임 구성의 요청이 온 것은 설정 오류다
            log_error("replier(" + name_ + "): 이름 또는 형식이 다른 요청");
            return {};
        }
        const std::shared_ptr<void> request = request_type_.create();
        std::uint64_t sequence = kNoSequence;
        if (!request_codec_.decode(wire->body, wire->binary, request.get(), sequence)) {
            log_error("replier(" + name_ + "): 형식이 맞지 않는 요청");
            return {};
        }
        const std::shared_ptr<void> reply = reply_type_.create();
        const RequestHandler handler = handler_;
        handler(request.get(), reply.get());

        // handler 가 이 객체를 없앴어도 Core 는 살아 있으므로 멤버를 읽을 수 있다
        Bytes body;
        Bytes binary;
        if (!reply_codec_.encode(reply.get(), kNoSequence, body, binary) || !binary.empty()) {
            log_error("replier(" + name_ + "): 응답을 전송 형식으로 바꾸지 못함");
            return {};
        }
        return body;
    }

    ZmqPoller& poller_;
    std::string name_;
    bus::MessageType request_type_;
    ZmqCodec request_codec_;
    bus::MessageType reply_type_;
    ZmqCodec reply_codec_;
    zmq::socket_t socket_;
    RequestHandler handler_;
    std::uint64_t registration_ = 0;  // serve() 를 부를 때마다 증가
    bool closed_ = false;
    core::Subscription poll_;
};

class ZmqReplier : public bus::IReplier {
public:
    explicit ZmqReplier(std::shared_ptr<ReplierCore> core) : core_(std::move(core)) {}
    ~ZmqReplier() override { core_->close(); }
    ZmqReplier(const ZmqReplier&) = delete;
    ZmqReplier& operator=(const ZmqReplier&) = delete;
    ZmqReplier(ZmqReplier&&) = delete;
    ZmqReplier& operator=(ZmqReplier&&) = delete;

    std::string_view name() const override { return core_->name(); }
    core::Subscription serve(RequestHandler handler) override {
        return core_->serve(std::move(handler));
    }

private:
    std::shared_ptr<ReplierCore> core_;
};

}  // namespace

// ---------------------------------------------------------------------------------------------

struct ZmqBus::Shared {
    Shared(zmq::context_t& zmq_context, ZmqPoller& zmq_poller, const core::IClock& bus_clock,
           ZmqEndpointTable endpoint_table, ZmqCodecTable codec_table)
        : context(zmq_context),
          poller(zmq_poller),
          clock(bus_clock),
          endpoints(std::move(endpoint_table)),
          codecs(std::move(codec_table)) {}

    // 이름의 엔드포인트. 빈 이름이거나 주소표에 없으면 이유를 남기고 nullopt
    std::optional<std::string> endpoint_of(std::string_view kind, std::string_view name) const {
        const std::string label = std::string(kind) + "(" + std::string(name) + ")";
        if (name.empty()) {
            log_error(label + ": 빈 이름");
            return std::nullopt;
        }
        const auto found = endpoints.find(name);
        if (found == endpoints.end()) {
            log_error(label + ": 주소표에 없는 이름");
            return std::nullopt;
        }
        return found->second;
    }

    // 타입의 변환 함수. 변환 표에 없거나 함수가 비어 있으면 이유를 남기고 nullptr
    const ZmqCodec* codec_of(std::string_view kind, std::string_view name,
                             const bus::MessageType& type) const {
        const auto found = codecs.find(type.type);
        if (found == codecs.end() || !found->second.encode || !found->second.decode) {
            log_error(std::string(kind) + "(" + std::string(name) + "): 변환 표에 없는 타입 " +
                      std::string(type.name));
            return nullptr;
        }
        return &found->second;
    }

    zmq::context_t& context;
    ZmqPoller& poller;
    const core::IClock& clock;
    ZmqEndpointTable endpoints;
    ZmqCodecTable codecs;
    // 엔드포인트 → 그 엔드포인트에 bind 한 PUB 소켓. 발행자가 모두 사라지면 소켓도 닫힌다
    std::map<std::string, std::weak_ptr<PubSocket>> pub_sockets;
};

ZmqBus::ZmqBus(zmq::context_t& context, ZmqPoller& poller, const core::IClock& clock,
               ZmqEndpointTable endpoints, ZmqCodecTable codecs)
    : shared_(std::make_shared<Shared>(context, poller, clock, std::move(endpoints),
                                       std::move(codecs))) {}

ZmqBus::~ZmqBus() = default;

std::unique_ptr<bus::ITopicPublisher> ZmqBus::create_publisher(std::string_view topic,
                                                               const bus::MessageType& type,
                                                               int queue_depth) {
    const std::string label = "create_publisher(" + std::string(topic) + ")";
    const std::optional<std::string> endpoint = shared_->endpoint_of("create_publisher", topic);
    if (!endpoint) {
        return nullptr;
    }
    const ZmqCodec* codec = shared_->codec_of("create_publisher", topic, type);
    if (codec == nullptr) {
        return nullptr;
    }
    if (queue_depth < 1) {
        log_error(label + ": queue_depth 가 1 미만");
        return nullptr;
    }
    try {
        std::shared_ptr<PubSocket> socket = shared_->pub_sockets[*endpoint].lock();
        if (!socket) {
            socket = std::make_shared<PubSocket>(shared_->context, *endpoint, queue_depth);
            shared_->pub_sockets[*endpoint] = socket;
        } else if (queue_depth > socket->queue_depth) {
            socket->grow_queue_depth(queue_depth);
        }
        return std::make_unique<ZmqTopicPublisher>(std::move(socket), topic, *codec);
    } catch (const zmq::error_t& error) {
        log_error(label + ": " + *endpoint + " bind 실패: " + error.what());
        return nullptr;
    }
}

std::unique_ptr<bus::ITopicSubscriber> ZmqBus::create_subscriber(std::string_view topic,
                                                                 const bus::MessageType& type,
                                                                 int queue_depth) {
    const std::string label = "create_subscriber(" + std::string(topic) + ")";
    const std::optional<std::string> endpoint = shared_->endpoint_of("create_subscriber", topic);
    if (!endpoint) {
        return nullptr;
    }
    const ZmqCodec* codec = shared_->codec_of("create_subscriber", topic, type);
    if (codec == nullptr) {
        return nullptr;
    }
    if (queue_depth < 1) {
        log_error(label + ": queue_depth 가 1 미만");
        return nullptr;
    }
    try {
        return std::make_unique<ZmqTopicSubscriber>(
            std::make_shared<SubscriberCore>(shared_->context, shared_->poller, shared_->clock,
                                             *endpoint, topic, type, *codec, queue_depth));
    } catch (const zmq::error_t& error) {
        log_error(label + ": " + *endpoint + " connect 실패: " + error.what());
        return nullptr;
    }
}

std::unique_ptr<bus::IRequester> ZmqBus::create_requester(std::string_view name,
                                                          const bus::MessageType& request,
                                                          const bus::MessageType& reply) {
    const std::optional<std::string> endpoint = shared_->endpoint_of("create_requester", name);
    if (!endpoint) {
        return nullptr;
    }
    const ZmqCodec* request_codec = shared_->codec_of("create_requester", name, request);
    const ZmqCodec* reply_codec = shared_->codec_of("create_requester", name, reply);
    if (request_codec == nullptr || reply_codec == nullptr) {
        return nullptr;
    }
    try {
        return std::make_unique<ZmqRequester>(
            std::make_shared<RequesterCore>(shared_->context, shared_->poller, shared_->clock,
                                            *endpoint, name, *request_codec, reply, *reply_codec));
    } catch (const zmq::error_t& error) {
        log_error("create_requester(" + std::string(name) + "): " + *endpoint +
                  " connect 실패: " + error.what());
        return nullptr;
    }
}

std::unique_ptr<bus::IReplier> ZmqBus::create_replier(std::string_view name,
                                                      const bus::MessageType& request,
                                                      const bus::MessageType& reply) {
    const std::optional<std::string> endpoint = shared_->endpoint_of("create_replier", name);
    if (!endpoint) {
        return nullptr;
    }
    const ZmqCodec* request_codec = shared_->codec_of("create_replier", name, request);
    const ZmqCodec* reply_codec = shared_->codec_of("create_replier", name, reply);
    if (request_codec == nullptr || reply_codec == nullptr) {
        return nullptr;
    }
    try {
        return std::make_unique<ZmqReplier>(
            std::make_shared<ReplierCore>(shared_->context, shared_->poller, *endpoint, name,
                                          request, *request_codec, reply, *reply_codec));
    } catch (const zmq::error_t& error) {
        log_error("create_replier(" + std::string(name) + "): " + *endpoint +
                  " bind 실패: " + error.what());
        return nullptr;
    }
}

}  // namespace argus::bus_zmq
