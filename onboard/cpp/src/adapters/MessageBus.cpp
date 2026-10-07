#include "argus/adapters/MessageBus.hpp"
namespace argus {
MessageBus::MessageBus(zmq::context_t &context, const std::string &endpoint)
    : socket_(context, zmq::socket_type::pub) {
    socket_.set(zmq::sockopt::linger, 0);
    socket_.set(zmq::sockopt::sndhwm, 10);
    socket_.connect(endpoint);
}
void MessageBus::publish(const std::string &topic, const std::string &header, const std::string *payload) {
    socket_.send(zmq::buffer(topic), zmq::send_flags::sndmore);
    socket_.send(zmq::buffer(header), payload ? zmq::send_flags::sndmore : zmq::send_flags::none);
    if (payload)
        socket_.send(zmq::buffer(*payload), zmq::send_flags::none);
}
std::string observation_message(const Observation &o) {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    auto field = [&](const char *key, const auto &value) {
        p.pack(std::string(key));
        p.pack(value);
    };
    auto optional = [&](const char *key, const auto &value) {
        p.pack(std::string(key));
        if (value)
            p.pack(*value);
        else
            p.pack_nil();
    };
    p.pack_map(23);
    field("schema", std::string("argus.perception.observations.v2"));
    field("source", o.source);
    field("frame_id", o.frame_id);
    field("seq", o.frame_id);
    field("image_width", o.image_width);
    field("image_height", o.image_height);
    field("model", o.model);
    field("inference_ms", o.inference_ms);
    field("coordinate_frame", std::string("camera_pixel"));
    field("observation_kind", std::string("IMAGE2D"));
    field("captured_unix_ns", o.timing.received_unix_ns);
    field("captured_monotonic_ns", o.timing.received_monotonic_ns);
    field("published_monotonic_ns", o.published_monotonic_ns);
    field("t_infer_us", o.infer_done_monotonic_ns / 1000);
    field("t_publish_us", o.published_monotonic_ns / 1000);
    field("received_unix_ns", o.timing.received_unix_ns);
    field("received_monotonic_ns", o.timing.received_monotonic_ns);
    optional("exposure_monotonic_ns", o.timing.exposure_monotonic_ns);
    optional("source_receive_monotonic_ns", o.timing.source_receive_monotonic_ns);
    field("clock_domain", o.timing.clock_domain);
    field("timestamp_source",
          std::string(o.timing.exposure_monotonic_ns ? "sensor_exposure" : "host_receive"));
    field("exposure_time_valid", bool(o.timing.exposure_monotonic_ns));
    p.pack(std::string("detections"));
    p.pack_array(o.full_boxes.size());
    for (const auto &b : o.full_boxes) {
        p.pack_map(8);
        field("class_id", b.class_id);
        field("label", b.label);
        field("confidence", b.confidence);
        field("x1", b.x1);
        field("y1", b.y1);
        field("x2", b.x2);
        field("y2", b.y2);
        p.pack(std::string("track_id"));
        p.pack_nil();
    }
    return {buffer.data(), buffer.size()};
}
void MessageBus::publish_observation(const Observation &o) {
    publish("perception/observations", observation_message(o));
}
void MessageBus::diagnostic(const std::string &source, const std::string &state, const std::string &detail,
                            const ClockStamp &stamp) {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    p.pack_map(6);
    p.pack("schema");
    p.pack("argus.diagnostics.event.v1");
    p.pack("source");
    p.pack(source);
    p.pack("state");
    p.pack(state);
    p.pack("detail");
    p.pack(detail);
    p.pack("unix_ns");
    p.pack(stamp.unix_ns);
    p.pack("monotonic_ns");
    p.pack(stamp.monotonic_ns);
    publish("diagnostics/events", {buffer.data(), buffer.size()});
}
} // namespace argus
