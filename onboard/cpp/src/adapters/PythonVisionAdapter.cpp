#include "argus/adapters/PythonVisionAdapter.hpp"
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <unistd.h>
namespace argus {
using Fields = std::map<std::string, msgpack::object>;
static Fields fields(const msgpack::object &object) {
    return object.as<Fields>();
}
static std::string request_header(const std::string &op, const std::string &session, const Frame *f) {
    msgpack::sbuffer buffer;
    msgpack::packer<msgpack::sbuffer> p(buffer);
    auto put = [&](const char *key, const auto &value) {
        p.pack(std::string(key));
        p.pack(value);
    };
    p.pack_map(f ? 14 : 3);
    put("schema", std::string("argus.ipc.perception.v1"));
    put("op", op);
    put("session_id", session);
    if (f) {
        put("frame_id", f->frame_id);
        put("width", f->width);
        put("height", f->height);
        put("stride", f->stride);
        put("pixel_format", f->pixel_format);
        put("received_unix_ns", f->timing.received_unix_ns);
        put("received_monotonic_ns", f->timing.received_monotonic_ns);
        put("clock_domain", f->timing.clock_domain);
        put("exposure_time_valid", bool(f->timing.exposure_monotonic_ns));
        p.pack("exposure_monotonic_ns");
        if (f->timing.exposure_monotonic_ns)
            p.pack(*f->timing.exposure_monotonic_ns);
        else
            p.pack_nil();
        p.pack("source_receive_monotonic_ns");
        if (f->timing.source_receive_monotonic_ns)
            p.pack(*f->timing.source_receive_monotonic_ns);
        else
            p.pack_nil();
    }
    return {buffer.data(), buffer.size()};
}
msgpack::object_handle PythonVisionAdapter::exchange(const std::string &header, const Frame *frame,
                                                     unsigned timeout) {
    {
        std::lock_guard<std::mutex> lock(relay_mutex_);
        if (relay_error_)
            std::rethrow_exception(relay_error_);
    }
    request_.set(zmq::sockopt::sndtimeo, int(timeout));
    auto sent = request_.send(zmq::buffer(header), frame ? zmq::send_flags::sndmore : zmq::send_flags::none);
    if (!sent)
        throw std::runtime_error("Python worker connection timed out");
    if (frame && !request_.send(zmq::buffer(*frame->pixels), zmq::send_flags::none))
        throw std::runtime_error("Python worker input send failed");
    zmq::pollitem_t item{request_.handle(), 0, ZMQ_POLLIN, 0};
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            zmq::poll(&item, 1, std::chrono::milliseconds(50));
        } catch (const zmq::error_t &ex) {
            if (ex.num() == EINTR)
                continue;
            throw;
        }
        if (item.revents & ZMQ_POLLIN) {
            zmq::message_t reply;
            if (!request_.recv(reply))
                throw std::runtime_error("Python reply missing");
            if (reply.more() || reply.size() > 4 * 1024 * 1024)
                throw std::runtime_error("invalid Python reply framing/size");
            auto decoded = msgpack::unpack(static_cast<const char *>(reply.data()), reply.size());
            auto map = fields(decoded.get());
            if (map.at("schema").as<std::string>() != "argus.ipc.perception.v1" ||
                map.at("session_id").as<std::string>() != session_)
                throw std::runtime_error("Python reply schema/session mismatch");
            if (!map.at("ok").as<bool>())
                throw std::runtime_error("Python worker: " + map.at("error").as<std::string>());
            return decoded;
        }
        if (!process_.running())
            throw std::runtime_error("Python worker exited unexpectedly");
    }
    throw std::runtime_error("Python inference response timed out");
}
PythonVisionAdapter::PythonVisionAdapter(zmq::context_t &context, const RuntimeOptions &o,
                                         const std::string &source, const std::string &session)
    : context_(context), request_(context, zmq::socket_type::req), session_(session), source_(source),
      publish_endpoint_(o.publish_endpoint), timeout_ms_(o.worker_timeout_ms) {
    try {
        char path[] = "/tmp/argus-vision-XXXXXX";
        if (!mkdtemp(path))
            throw std::runtime_error("cannot create private IPC directory");
        directory_ = path;
        request_.set(zmq::sockopt::linger, 0);
        request_.set(zmq::sockopt::sndhwm, 1);
        request_.set(zmq::sockopt::rcvhwm, 1);
        request_.set(zmq::sockopt::immediate, 1);
        request_.connect("ipc://" + directory_ + "/requests");
        preview_relay_ = std::thread(&PythonVisionAdapter::relay, this);
        std::vector<std::string> args{o.python,
                                      "-m",
                                      o.worker_module,
                                      "--endpoint",
                                      "ipc://" + directory_ + "/requests",
                                      "--preview-endpoint",
                                      "ipc://" + directory_ + "/preview",
                                      "--session-id",
                                      session,
                                      "--source",
                                      source,
                                      "--backend",
                                      o.backend,
                                      "--labels",
                                      o.labels,
                                      "--score-threshold",
                                      std::to_string(o.score_threshold),
                                      "--ir-preprocess",
                                      o.ir_preprocess,
                                      "--preview-max-fps",
                                      std::to_string(o.preview_fps),
                                      "--jpeg-quality",
                                      std::to_string(o.jpeg_quality),
                                      "--npu-input-max-fps",
                                      std::to_string(o.npu_input_fps),
                                      "--npu-input-jpeg-quality",
                                      std::to_string(o.npu_jpeg_quality)};
        if (!o.hef.empty()) {
            args.push_back("--hef");
            args.push_back(o.hef);
        }
        if (o.preview)
            args.push_back("--publish-preview");
        if (o.npu_input)
            args.push_back("--publish-npu-input");
        process_.start(args);
        auto ready = exchange(request_header("ready", session_, nullptr), nullptr, o.worker_startup_ms);
        auto map = fields(ready.get());
        if (map.at("input_width").as<unsigned>() == 0 || map.at("input_height").as<unsigned>() == 0)
            throw std::runtime_error("invalid detector input shape");
    } catch (...) {
        stop();
        throw;
    }
}
DetectionResult PythonVisionAdapter::infer(const Frame &frame) {
    frame.validate();
    SystemClock clock;
    auto begin = clock.now().monotonic_ns;
    auto reply = exchange(request_header("infer", session_, &frame), &frame, timeout_ms_);
    auto map = fields(reply.get());
    DetectionResult result;
    result.frame_id = map.at("frame_id").as<std::uint64_t>();
    result.session_id = map.at("session_id").as<std::string>();
    if (result.frame_id != frame.frame_id)
        throw std::runtime_error("Python reply frame mismatch");
    if (map.at("coordinate_frame").as<std::string>() != "camera_pixel")
        throw std::runtime_error("Python reply coordinate mismatch");
    result.model = map.at("model").as<std::string>();
    result.infer_done_monotonic_ns = map.at("infer_done_monotonic_ns").as<std::uint64_t>();
    result.preprocess_ms = map.at("preprocess_ms").as<double>();
    result.inference_ms = map.at("inference_ms").as<double>();
    auto elapsed = (clock.now().monotonic_ns - begin) / 1000000.0;
    result.ipc_ms = std::max(0.0, elapsed - result.preprocess_ms - result.inference_ms);
    auto rows = map.at("detections").as<std::vector<msgpack::object>>();
    if (rows.size() > 10000)
        throw std::runtime_error("too many detections");
    for (const auto &row : rows) {
        auto b = fields(row);
        result.full_boxes.push_back({b.at("class_id").as<int>(), b.at("label").as<std::string>(),
                                     b.at("confidence").as<double>(), b.at("x1").as<int>(),
                                     b.at("y1").as<int>(), b.at("x2").as<int>(), b.at("y2").as<int>()});
    }
    return result;
}
void PythonVisionAdapter::relay() {
    try {
        zmq::socket_t socket(context_, zmq::socket_type::pull);
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::rcvhwm, 2);
        socket.set(zmq::sockopt::maxmsgsize, std::int64_t(16 * 1024 * 1024));
        socket.bind("ipc://" + directory_ + "/preview");
        MessageBus bus(context_, publish_endpoint_);
        while (!stopping_) {
            zmq::pollitem_t item{socket.handle(), 0, ZMQ_POLLIN, 0};
            zmq::poll(&item, 1, std::chrono::milliseconds(50));
            if (!(item.revents & ZMQ_POLLIN))
                continue;
            std::vector<std::string> parts;
            do {
                zmq::message_t message;
                if (!socket.recv(message))
                    throw std::runtime_error("preview frame missing");
                parts.emplace_back(static_cast<const char *>(message.data()), message.size());
                if (parts.size() > 3)
                    throw std::runtime_error("invalid preview multipart");
                if (!message.more())
                    break;
            } while (true);
            if (parts.size() != 3 ||
                (parts[0] != "vision/overlay/jpeg" && parts[0] != "vision/npu_input/jpeg"))
                throw std::runtime_error("invalid preview topic");
            auto decoded = msgpack::unpack(parts[1].data(), parts[1].size());
            auto header = fields(decoded.get());
            if (header.at("session_id").as<std::string>() != session_ ||
                header.at("source").as<std::string>() != source_)
                throw std::runtime_error("preview source/session mismatch");
            bus.publish(parts[0], parts[1], &parts[2]);
        }
    } catch (...) {
        std::lock_guard<std::mutex> guard(relay_mutex_);
        relay_error_ = std::current_exception();
    }
}
void PythonVisionAdapter::stop() noexcept {
    if (stopping_.exchange(true))
        return;
    if (preview_relay_.joinable())
        preview_relay_.join();
    try {
        if (process_.running())
            exchange(request_header("shutdown", session_, nullptr), nullptr, 200);
    } catch (...) {
    }
    try {
        request_.close();
    } catch (...) {
    }
    process_.stop();
    if (!directory_.empty()) {
        std::error_code ignored;
        std::filesystem::remove_all(directory_, ignored);
    }
}
PythonVisionAdapter::~PythonVisionAdapter() {
    stop();
}
} // namespace argus
