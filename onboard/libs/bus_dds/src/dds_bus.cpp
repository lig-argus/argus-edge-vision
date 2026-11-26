#include "argus/bus_dds/dds_bus.hpp"

#include <chrono>
#include <fastdds/dds/core/ReturnCode.hpp>
#include <fastdds/dds/core/status/StatusMask.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/bus/replier.hpp"
#include "argus/bus/requester.hpp"
#include "argus/bus/topic_publisher.hpp"
#include "argus/bus/topic_subscriber.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "fastdds_specific.hpp"
#include "wakeup_pipe.hpp"

namespace argus::bus_dds {
namespace dds = eprosima::fastdds::dds;

namespace detail {

// Fast DDS 의 스레드에서도 부르므로 한 줄을 한 번에 기록한다 (std::cerr 는 동시 사용에 안전)
void log_error(const std::string& text) { std::cerr << ("[bus_dds] " + text + '\n'); }

struct DdsBusState {
    DdsBusState(core::IFdWatcher& w, core::ITimerScheduler& s, const core::IClock& c,
                DdsTypeTable t, DdsTopicQosTable q)
        : watcher(w), scheduler(s), clock(c), types(std::move(t)), topic_qos(std::move(q)) {}

    ~DdsBusState() {
        if (participant != nullptr) {
            participant->delete_contained_entities();
            dds::DomainParticipantFactory::get_instance()->delete_participant(participant);
        }
    }

    DdsBusState(const DdsBusState&) = delete;
    DdsBusState& operator=(const DdsBusState&) = delete;
    DdsBusState(DdsBusState&&) = delete;
    DdsBusState& operator=(DdsBusState&&) = delete;

    // 표에서 type 의 IDL 타입을 찾는다. 없으면 nullptr 이고 로그에 남긴다
    const DdsType* find_type(const bus::MessageType& type) const {
        const auto found = types.find(type.type);
        if (found == types.end()) {
            log_error(std::string("타입 표에 없는 타입: ") + std::string(type.name));
            return nullptr;
        }
        return &found->second;
    }

    // type 의 IDL 타입을 DomainParticipant 에 등록하고 그 TypeSupport 를 반환한다 (이미 등록했으면
    // 등록한 것). 등록하지 못하면 nullopt
    std::optional<dds::TypeSupport> register_type(const DdsType& type) {
        dds::TypeSupport support = type.make_type_support();
        const std::string name = support.get_type_name();
        const auto found = registered_types.find(name);
        if (found != registered_types.end()) {
            return found->second;
        }
        if (support.register_type(participant) != dds::RETCODE_OK) {
            log_error("IDL 타입 등록 실패: " + name);
            return std::nullopt;
        }
        registered_types.emplace(name, support);
        return support;
    }

    // 토픽 name 의 발행자 QoS 값. 표에 없으면 빈 값 (Deadline 무한)
    DdsTopicQos qos_of(const std::string& name) const {
        const auto found = topic_qos.find(name);
        return found != topic_qos.end() ? found->second : DdsTopicQos{};
    }

    // 이름이 name 인 토픽을 반환한다. DomainParticipant 안에서 토픽 이름은 하나여야 하므로 같은
    // 이름의 엔티티끼리 토픽 하나를 공유하고, 마지막 사용자가 release_topic() 을 부를 때 지운다
    dds::Topic* acquire_topic(const std::string& name, const dds::TypeSupport& support) {
        const auto found = topics.find(name);
        if (found != topics.end()) {
            if (found->second.topic->get_type_name() != support.get_type_name()) {
                log_error("토픽 " + name +
                          " 의 타입이 다름: " + found->second.topic->get_type_name());
                return nullptr;
            }
            ++found->second.users;
            return found->second.topic;
        }
        dds::Topic* const topic =
            participant->create_topic(name, support.get_type_name(), dds::TOPIC_QOS_DEFAULT);
        if (topic == nullptr) {
            log_error("토픽 생성 실패: " + name);
            return nullptr;
        }
        topics.emplace(name, TopicUse{topic, 1});
        return topic;
    }

    void release_topic(const std::string& name) {
        const auto found = topics.find(name);
        if (found == topics.end() || --found->second.users > 0) {
            return;
        }
        participant->delete_topic(found->second.topic);
        topics.erase(found);
    }

    struct TopicUse {
        dds::Topic* topic = nullptr;
        std::int32_t users = 0;
    };

    core::IFdWatcher& watcher;
    core::ITimerScheduler& scheduler;
    const core::IClock& clock;
    DdsTypeTable types;
    DdsTopicQosTable topic_qos;
    dds::DomainParticipant* participant = nullptr;
    dds::Publisher* publisher = nullptr;
    dds::Subscriber* subscriber = nullptr;
    std::map<std::string, dds::TypeSupport> registered_types;
    std::map<std::string, TopicUse> topics;
};

}  // namespace detail

namespace {

using detail::DdsBusState;
using detail::log_error;
using detail::SampleIdentity;

constexpr std::int64_t kNsPerMs = 1'000'000;

// 요청과 응답 토픽의 History depth. 요청-응답에는 bus 인터페이스의 queue_depth 가 없으므로,
// 응답하는 쪽이 처리하기 전에 여러 요청하는 쪽의 요청이 쌓여도 버리지 않을 만큼으로 정한다
constexpr std::int32_t kRequestReplyDepth = 16;

// 토픽 하나의 DataWriter 와 표본 버퍼. 발행하는 쪽의 Deadline 초과와 구독자와의 QoS 불일치는
// Fast DDS 의 스레드에서 로그에 남긴다
class Writer : public dds::DataWriterListener {
public:
    // qos 는 토픽의 Deadline. 요청-응답의 토픽은 빈 값
    static std::unique_ptr<Writer> create(std::shared_ptr<DdsBusState> state,
                                          const std::string& topic_name, const DdsType& type,
                                          std::int32_t depth, const DdsTopicQos& qos) {
        const std::optional<dds::TypeSupport> support = state->register_type(type);
        if (!support.has_value()) {
            return nullptr;
        }
        dds::Topic* const topic = state->acquire_topic(topic_name, support.value());
        if (topic == nullptr) {
            return nullptr;
        }
        std::unique_ptr<Writer> writer(
            new Writer(std::move(state), topic_name, type, support.value()));
        const dds::StatusMask mask = dds::StatusMask::offered_deadline_missed()
                                     << dds::StatusMask::offered_incompatible_qos();
        writer->writer_ = writer->state_->publisher->create_datawriter(
            topic, detail::writer_qos(depth, qos), writer.get(), mask);
        if (writer->writer_ == nullptr) {
            log_error("DataWriter 생성 실패: " + topic_name);
            return nullptr;
        }
        return writer;
    }

    ~Writer() override {
        if (writer_ != nullptr) {
            writer_->set_listener(nullptr);
            state_->publisher->delete_datawriter(writer_);
        }
        support_.delete_data(sample_);
        state_->release_topic(topic_name_);
    }

    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    Writer(Writer&&) = delete;
    Writer& operator=(Writer&&) = delete;

    const std::string& topic_name() const { return topic_name_; }

    // 아래 둘은 Fast DDS 의 스레드에서 불린다. 로그만 남긴다
    void on_offered_deadline_missed(dds::DataWriter* /*writer*/,
                                    const dds::OfferedDeadlineMissedStatus& /*status*/) override {
        log_error("Deadline 초과 (발행 지연): " + topic_name_);
    }
    void on_offered_incompatible_qos(dds::DataWriter* /*writer*/,
                                     const dds::OfferedIncompatibleQosStatus& /*status*/) override {
        log_error("구독자와 QoS 불일치: " + topic_name_);
    }

    bool write(const void* value) {
        return fill(value) && writer_->write(sample_) == dds::RETCODE_OK;
    }

    std::optional<SampleIdentity> write_identified(const void* value) {
        if (!fill(value)) {
            return std::nullopt;
        }
        return detail::write_identified(*writer_, sample_);
    }

    bool write_related(const void* value, const SampleIdentity& request) {
        return fill(value) && detail::write_related(*writer_, sample_, request);
    }

private:
    Writer(std::shared_ptr<DdsBusState> state, std::string topic_name, const DdsType& type,
           dds::TypeSupport support)
        : state_(std::move(state)),
          topic_name_(std::move(topic_name)),
          type_(type),
          support_(std::move(support)),
          sample_(support_.create_data()) {}

    bool fill(const void* value) {
        if (!type_.to_idl(value, sample_)) {
            log_error("IDL 변환 실패: " + topic_name_);
            return false;
        }
        return true;
    }

    std::shared_ptr<DdsBusState> state_;
    std::string topic_name_;
    const DdsType& type_;  // state_->types 의 원소
    dds::TypeSupport support_;
    void* sample_;
    dds::DataWriter* writer_ = nullptr;
};

// 토픽 하나의 DataReader. Fast DDS 스레드의 수신 알림은 pipe 를 깨우기만 하고, IFdWatcher 에 등록한
// handler 가 process_events() 안에서 표본을 꺼내 on_sample 을 부른다. 발행자와의 QoS 불일치는
// Fast DDS 의 스레드에서 로그에 남긴다
class Reader : public dds::DataReaderListener {
public:
    using SampleHandler = std::function<void(const void* sample, const dds::SampleInfo& info)>;

    static std::unique_ptr<Reader> create(const std::shared_ptr<DdsBusState>& state,
                                          const std::string& topic_name, const DdsType& type,
                                          std::int32_t depth, SampleHandler on_sample) {
        const std::optional<dds::TypeSupport> support = state->register_type(type);
        if (!support.has_value()) {
            return nullptr;
        }
        dds::Topic* const topic = state->acquire_topic(topic_name, support.value());
        if (topic == nullptr) {
            return nullptr;
        }
        std::unique_ptr<Reader> reader(
            new Reader(state, topic_name, support.value(), std::move(on_sample)));
        if (!reader->pipe_.valid()) {
            log_error("pipe 생성 실패: " + topic_name);
            return nullptr;
        }
        // 등록은 DataReader 를 만들기 전에 한다. 만든 직후 도착한 표본의 알림도 처리되도록
        Reader* const raw = reader.get();
        reader->registration_ =
            state->watcher.add_fd(reader->pipe_.read_fd(), [raw] { raw->take_all(); });
        if (!reader->registration_.active()) {
            return nullptr;
        }
        const dds::StatusMask mask = dds::StatusMask::data_available()
                                     << dds::StatusMask::requested_incompatible_qos();
        reader->reader_ = state->subscriber->create_datareader(topic, detail::reader_qos(depth),
                                                               reader.get(), mask);
        if (reader->reader_ == nullptr) {
            log_error("DataReader 생성 실패: " + topic_name);
            return nullptr;
        }
        return reader;
    }

    ~Reader() override {
        registration_.reset();
        if (reader_ != nullptr) {
            reader_->set_listener(nullptr);
            state_->subscriber->delete_datareader(reader_);
        }
        support_.delete_data(sample_);
        state_->release_topic(topic_name_);
    }

    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    Reader(Reader&&) = delete;
    Reader& operator=(Reader&&) = delete;

    // Fast DDS 의 수신 스레드에서 불린다. pipe 에 기록하는 일만 한다
    void on_data_available(dds::DataReader* /*reader*/) override { pipe_.notify(); }

    // Fast DDS 의 스레드에서 불린다. 로그만 남긴다
    void on_requested_incompatible_qos(
        dds::DataReader* /*reader*/,
        const dds::RequestedIncompatibleQosStatus& /*status*/) override {
        log_error("발행자와 QoS 불일치: " + topic_name_);
    }

private:
    Reader(std::shared_ptr<DdsBusState> state, std::string topic_name, dds::TypeSupport support,
           SampleHandler on_sample)
        : state_(std::move(state)),
          topic_name_(std::move(topic_name)),
          support_(std::move(support)),
          sample_(support_.create_data()),
          on_sample_(std::move(on_sample)) {}

    void take_all() {
        pipe_.drain();
        dds::SampleInfo info;
        while (reader_ != nullptr && reader_->take_next_sample(sample_, &info) == dds::RETCODE_OK) {
            if (info.valid_data) {
                on_sample_(sample_, info);
            }
        }
    }

    std::shared_ptr<DdsBusState> state_;
    std::string topic_name_;
    dds::TypeSupport support_;
    void* sample_;
    SampleHandler on_sample_;
    detail::WakeupPipe pipe_;
    core::Subscription registration_;  // pipe 의 IFdWatcher 등록
    dds::DataReader* reader_ = nullptr;
};

// 표본을 값으로 바꾼다. 실패하면 nullptr 이고 로그에 남긴다
std::shared_ptr<void> to_value(const DdsType& type, const bus::MessageType& message_type,
                               const void* sample, const std::string& topic_name) {
    std::shared_ptr<void> value = message_type.create();
    if (!type.from_idl(sample, value.get())) {
        log_error("IDL 표본 변환 실패, 폐기: " + topic_name);
        return nullptr;
    }
    return value;
}

class DdsTopicPublisher : public bus::ITopicPublisher {
public:
    explicit DdsTopicPublisher(std::unique_ptr<Writer> writer) : writer_(std::move(writer)) {}

    std::string_view topic() const override { return writer_->topic_name(); }

    bool publish(const void* message) override { return writer_->write(message); }

private:
    std::unique_ptr<Writer> writer_;
};

class DdsTopicSubscriber : public bus::ITopicSubscriber {
public:
    using Handlers = core::HandlerList<const void*, const bus::MessageInfo&>;

    DdsTopicSubscriber(std::string topic, std::shared_ptr<Handlers> handlers,
                       std::unique_ptr<Reader> reader)
        : topic_(std::move(topic)), handlers_(std::move(handlers)), reader_(std::move(reader)) {}

    std::string_view topic() const override { return topic_; }

    [[nodiscard]] core::Subscription subscribe(Handler handler) override {
        if (!handler) {
            return {};
        }
        return handlers_->subscribe(std::move(handler));
    }

private:
    std::string topic_;
    std::shared_ptr<Handlers> handlers_;
    std::unique_ptr<Reader> reader_;
};

// 요청하는 쪽이 응답을 기다리는 요청들
struct PendingRequests {
    struct Entry {
        std::optional<SampleIdentity> identity;  // 보내지 못한 요청은 nullopt (timeout 으로 끝남)
        bus::IRequester::ReplyHandler on_reply;
        core::Subscription timer;
    };

    // id 의 요청을 끝내고 on_reply 를 reply 로 부른다. 이미 끝났으면 아무것도 하지 않는다
    void finish(std::uint64_t id, const void* reply) {
        const auto found = entries.find(id);
        if (found == entries.end()) {
            return;
        }
        // handler 가 다른 요청을 보내거나 해제해도 안전하도록 먼저 꺼내고 지운 뒤 부른다
        const bus::IRequester::ReplyHandler on_reply = std::move(found->second.on_reply);
        entries.erase(found);
        on_reply(reply);
    }

    std::optional<std::uint64_t> find(const SampleIdentity& request) const {
        for (const auto& [id, entry] : entries) {
            if (entry.identity.has_value() && entry.identity.value() == request) {
                return id;
            }
        }
        return std::nullopt;
    }

    std::map<std::uint64_t, Entry> entries;
    std::uint64_t next_id = 1;
};

class DdsRequester : public bus::IRequester {
public:
    DdsRequester(std::shared_ptr<DdsBusState> state, std::string name,
                 std::shared_ptr<PendingRequests> pending, std::unique_ptr<Writer> request_writer,
                 std::unique_ptr<Reader> reply_reader)
        : state_(std::move(state)),
          name_(std::move(name)),
          pending_(std::move(pending)),
          request_writer_(std::move(request_writer)),
          reply_reader_(std::move(reply_reader)) {}

    std::string_view name() const override { return name_; }

    [[nodiscard]] core::Subscription request(const void* request, std::chrono::milliseconds timeout,
                                             ReplyHandler on_reply) override {
        if (!on_reply) {
            return {};
        }
        const std::uint64_t id = pending_->next_id++;
        PendingRequests::Entry& entry = pending_->entries[id];
        entry.on_reply = std::move(on_reply);
        entry.identity = request_writer_->write_identified(request);
        if (!entry.identity.has_value()) {
            log_error("요청 송신 실패: " + name_);
        }
        const std::weak_ptr<PendingRequests> weak = pending_;
        entry.timer = state_->scheduler.add_timer_at(
            state_->clock.now_monotonic_ns() + timeout.count() * kNsPerMs, [weak, id] {
                if (const auto locked = weak.lock()) {
                    locked->finish(id, nullptr);
                }
            });
        return core::Subscription([weak, id] {
            if (const auto locked = weak.lock()) {
                locked->entries.erase(id);
            }
        });
    }

private:
    std::shared_ptr<DdsBusState> state_;
    std::string name_;
    std::shared_ptr<PendingRequests> pending_;
    std::unique_ptr<Writer> request_writer_;
    std::unique_ptr<Reader> reply_reader_;
};

// 응답하는 쪽의 handler. 다시 등록하면 앞의 등록을 대신한다
struct ServeState {
    bus::IReplier::RequestHandler handler;
    std::uint64_t id = 0;  // 마지막 등록의 번호
};

class DdsReplier : public bus::IReplier {
public:
    DdsReplier(std::string name, std::shared_ptr<ServeState> serve,
               std::shared_ptr<Writer> reply_writer, std::unique_ptr<Reader> request_reader)
        : name_(std::move(name)),
          serve_(std::move(serve)),
          reply_writer_(std::move(reply_writer)),
          request_reader_(std::move(request_reader)) {}

    std::string_view name() const override { return name_; }

    [[nodiscard]] core::Subscription serve(RequestHandler handler) override {
        if (!handler) {
            return {};
        }
        serve_->handler = std::move(handler);
        const std::uint64_t id = ++serve_->id;
        const std::weak_ptr<ServeState> weak = serve_;
        return core::Subscription([weak, id] {
            const auto locked = weak.lock();
            if (locked && locked->id == id) {
                locked->handler = nullptr;  // 다른 등록이 대신했으면 유지한다
            }
        });
    }

private:
    std::string name_;
    std::shared_ptr<ServeState> serve_;
    std::shared_ptr<Writer> reply_writer_;

    // 요청 표본의 handler 가 응답 DataWriter 를 사용하므로 DataWriter 보다 뒤에 선언하여 먼저
    // 지운다
    std::unique_ptr<Reader> request_reader_;
};

bool valid_name(std::string_view name) {
    if (name.empty()) {
        log_error("빈 이름");
        return false;
    }
    return true;
}

}  // namespace

std::unique_ptr<DdsBus> DdsBus::create(core::IFdWatcher& watcher, core::ITimerScheduler& scheduler,
                                       const core::IClock& clock, std::uint32_t domain_id,
                                       DdsTypeTable types, DdsTopicQosTable topic_qos) {
    auto state = std::make_shared<DdsBusState>(watcher, scheduler, clock, std::move(types),
                                               std::move(topic_qos));
    state->participant = dds::DomainParticipantFactory::get_instance()->create_participant(
        domain_id, dds::PARTICIPANT_QOS_DEFAULT);
    if (state->participant == nullptr) {
        log_error("DomainParticipant 생성 실패 (domain " + std::to_string(domain_id) + ")");
        return nullptr;
    }
    state->publisher = state->participant->create_publisher(dds::PUBLISHER_QOS_DEFAULT);
    state->subscriber = state->participant->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT);
    if (state->publisher == nullptr || state->subscriber == nullptr) {
        log_error("Publisher 또는 Subscriber 생성 실패");
        return nullptr;
    }
    return std::unique_ptr<DdsBus>(new DdsBus(std::move(state)));
}

DdsBus::DdsBus(std::shared_ptr<detail::DdsBusState> state) : state_(std::move(state)) {}

DdsBus::~DdsBus() = default;

std::unique_ptr<bus::ITopicPublisher> DdsBus::create_publisher(std::string_view topic,
                                                               const bus::MessageType& type,
                                                               std::int32_t queue_depth) {
    const DdsType* const dds_type = state_->find_type(type);
    if (!valid_name(topic) || dds_type == nullptr || queue_depth < 1) {
        return nullptr;
    }
    auto writer = Writer::create(state_, std::string(topic), *dds_type, queue_depth,
                                 state_->qos_of(std::string(topic)));
    if (!writer) {
        return nullptr;
    }
    return std::make_unique<DdsTopicPublisher>(std::move(writer));
}

std::unique_ptr<bus::ITopicSubscriber> DdsBus::create_subscriber(std::string_view topic,
                                                                 const bus::MessageType& type,
                                                                 std::int32_t queue_depth) {
    const DdsType* const dds_type = state_->find_type(type);
    if (!valid_name(topic) || dds_type == nullptr || queue_depth < 1) {
        return nullptr;
    }
    const std::string topic_name(topic);
    auto handlers = std::make_shared<DdsTopicSubscriber::Handlers>();
    const std::weak_ptr<DdsTopicSubscriber::Handlers> weak = handlers;
    auto reader = Reader::create(
        state_, topic_name, *dds_type, queue_depth,
        // 이름과 타입은 init-capture 로 const 가 아닌 사본으로 잡는다 (const 멤버가 있으면 람다를
        // 이동할 때 복사가 일어나 예외가 날 수 있다)
        [weak, dds_type, message_type = type, name = topic_name](const void* sample,
                                                                 const dds::SampleInfo& info) {
            const auto locked = weak.lock();
            if (!locked) {
                return;
            }
            const std::shared_ptr<void> value = to_value(*dds_type, message_type, sample, name);
            if (value) {
                locked->notify(value.get(), bus::MessageInfo{detail::sequence_of(info)});
            }
        });
    if (!reader) {
        return nullptr;
    }
    return std::make_unique<DdsTopicSubscriber>(topic_name, std::move(handlers), std::move(reader));
}

std::unique_ptr<bus::IRequester> DdsBus::create_requester(std::string_view name,
                                                          const bus::MessageType& request,
                                                          const bus::MessageType& reply) {
    const DdsType* const request_type = state_->find_type(request);
    const DdsType* const reply_type = state_->find_type(reply);
    if (!valid_name(name) || request_type == nullptr || reply_type == nullptr) {
        return nullptr;
    }
    const std::string base(name);
    auto writer =
        Writer::create(state_, base + "_Request", *request_type, kRequestReplyDepth, DdsTopicQos{});
    if (!writer) {
        return nullptr;
    }
    auto pending = std::make_shared<PendingRequests>();
    const std::weak_ptr<PendingRequests> weak = pending;
    const std::string reply_topic = base + "_Reply";
    auto reader = Reader::create(
        state_, reply_topic, *reply_type, kRequestReplyDepth,
        [weak, reply_type, reply_message = reply, name = reply_topic](const void* sample,
                                                                      const dds::SampleInfo& info) {
            const auto locked = weak.lock();
            if (!locked) {
                return;
            }
            // 같은 이름의 다른 요청하는 쪽의 응답도 오므로 자기 요청의 식별자가 붙은 것만 받는다
            const std::optional<std::uint64_t> id = locked->find(detail::related_identity_of(info));
            if (!id.has_value()) {
                return;
            }
            const std::shared_ptr<void> value = to_value(*reply_type, reply_message, sample, name);
            if (value) {
                locked->finish(id.value(), value.get());
            }
        });
    if (!reader) {
        return nullptr;
    }
    return std::make_unique<DdsRequester>(state_, base, std::move(pending), std::move(writer),
                                          std::move(reader));
}

std::unique_ptr<bus::IReplier> DdsBus::create_replier(std::string_view name,
                                                      const bus::MessageType& request,
                                                      const bus::MessageType& reply) {
    const DdsType* const request_type = state_->find_type(request);
    const DdsType* const reply_type = state_->find_type(reply);
    if (!valid_name(name) || request_type == nullptr || reply_type == nullptr) {
        return nullptr;
    }
    const std::string base(name);
    auto writer =
        Writer::create(state_, base + "_Reply", *reply_type, kRequestReplyDepth, DdsTopicQos{});
    if (!writer) {
        return nullptr;
    }
    // 요청 표본의 handler 가 응답 DataWriter 를 사용하므로 공유한다
    const std::shared_ptr<Writer> reply_writer(std::move(writer));
    auto serve = std::make_shared<ServeState>();
    const std::weak_ptr<ServeState> weak_serve = serve;
    const std::weak_ptr<Writer> weak_writer = reply_writer;
    const std::string request_topic = base + "_Request";
    auto reader = Reader::create(
        state_, request_topic, *request_type, kRequestReplyDepth,
        [weak_serve, weak_writer, request_type, request_message = request, reply_message = reply,
         name = request_topic](const void* sample, const dds::SampleInfo& info) {
            const auto serve_state = weak_serve.lock();
            const auto reply_writer_locked = weak_writer.lock();
            // handler 가 없으면 응답하지 않는다 (요청하는 쪽은 timeout 으로 안다)
            if (!serve_state || !serve_state->handler || !reply_writer_locked) {
                return;
            }
            const std::shared_ptr<void> request_value =
                to_value(*request_type, request_message, sample, name);
            if (!request_value) {
                return;
            }
            const std::shared_ptr<void> reply_value = reply_message.create();
            // handler 가 자기 등록을 해제해도 안전하도록 복사해서 부른다
            const bus::IReplier::RequestHandler handler = serve_state->handler;
            handler(request_value.get(), reply_value.get());
            if (!reply_writer_locked->write_related(reply_value.get(), detail::identity_of(info))) {
                log_error("응답 송신 실패: " + name);
            }
        });
    if (!reader) {
        return nullptr;
    }
    return std::make_unique<DdsReplier>(base, std::move(serve), reply_writer, std::move(reader));
}

}  // namespace argus::bus_dds
