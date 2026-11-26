#include <iostream>
#include <limits>

#include "argus/common/include/observation.hpp"
#include "argus/perception/include/box_postprocessor.hpp"
#include "argus/perception/include/frame_queue.hpp"
#include "argus/runtime/include/environment_profile.hpp"
#include "argus/transport/include/message_bus.hpp"
static void require(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
template <class F>
static void rejects(F&& function) {
    bool rejected = false;
    try {
        function();
    } catch (const std::exception&) {
        rejected = true;
    }
    require(rejected, "invalid contract accepted");
}
int main() {
    try {
        argus::FrameTiming receipt{1000, 100, std::nullopt, std::nullopt, "host_monotonic"};
        rejects([&] { receipt.require_exposure_time(); });
        receipt.exposure_monotonic_ns = 50;
        require(receipt.require_exposure_time() == 50, "exposure lost");
        receipt.exposure_monotonic_ns = 101;
        rejects([&] { receipt.validate(); });
        receipt.exposure_monotonic_ns.reset();
        argus::Frame frame{
            7, 4, 4, 12, "BGR8", std::make_shared<std::vector<std::uint8_t>>(48), receipt};
        frame.validate();
        auto invalid = frame;
        invalid.stride = 8;
        rejects([&] { invalid.validate(); });
        argus::FrameQueue queue(2);
        queue.put(frame);
        frame.frame_id = 8;
        queue.put(frame);
        frame.frame_id = 9;
        queue.put(frame);
        queue.close();
        require(queue.dropped() == 1, "queue did not drop oldest");
        argus::Frame out;
        require(queue.take(out, std::chrono::milliseconds(1)) && out.frame_id == 8,
                "FIFO/drop-oldest changed");
        require(
            queue.take(out, std::chrono::milliseconds(1)) && out.frame_id == 9 && queue.drained(),
            "queue did not drain");
        rejects([] { argus::FrameQueue invalid(0); });
        argus::FullBox box{0, "target", 0.9, 0, 0, 3, 3};
        argus::EnvironmentProfile profile{"REAL_IR", "ir_sdk",        "hailo_yolox",
                                          "ucmc",    "mavsdk_serial", "camera_pixel"};
        argus::validate_profile(profile);
        for (const auto& name : {"bytetrack_cmc", "ocsort_cmc", "hybridsort_hmiou_off_cmc",
                                 "hybridsort_hmiou_on_cmc"}) {
            profile.tracker = name;
            rejects([&] { argus::validate_profile(profile); });
        }
        profile.tracker = "ucmc";
        profile.coordinate_frame = "gazebo_world_enu";
        rejects([&] { argus::validate_profile(profile); });
        frame.frame_id = 7;
        argus::DetectionResult result{7, 120, "session", "model", 1, 2, 3, {box}};
        argus::BoxPostprocessor post;
        post.validate(result, frame, "session");
        result.frame_id = 8;
        rejects([&] { post.validate(result, frame, "session"); });
        result.frame_id = 7;
        result.full_boxes[0].x2 = 4;
        rejects([&] { post.validate(result, frame, "session"); });
        result.full_boxes[0] = box;
        result.full_boxes[0].confidence = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { post.validate(result, frame, "session"); });
        argus::Observation observation;
        observation.frame_id = 7;
        observation.session_id = "session";
        observation.timing = receipt;
        observation.full_boxes = {box};
        auto bytes = argus::observation_message(observation);
        auto decoded = msgpack::unpack(bytes.data(), bytes.size());
        auto map = decoded.get().as<std::map<std::string, msgpack::object>>();
        require(map.size() == 23 &&
                    map.at("schema").as<std::string>() == "argus.perception.observations.v2",
                "observation schema/framing");
        require(map.at("seq").as<unsigned>() == 7 && !map.at("exposure_time_valid").as<bool>() &&
                    map.at("exposure_monotonic_ns").is_nil(),
                "frame identity/time falsified");
        auto detections = map.at("detections").as<std::vector<msgpack::object>>();
        auto b = detections[0].as<std::map<std::string, msgpack::object>>();
        require(b.size() == 8 && b.at("track_id").is_nil() && !b.count("hull_x1"),
                "full box contract changed");
        std::cout << "Frame/queue/exposure/profile/full-box/MessagePack contracts passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
