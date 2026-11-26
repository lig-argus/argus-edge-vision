#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <iostream>

#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/perception/geometry/include/pi_ground_projector.hpp"
#include "argus/perception/geometry/include/pinhole_distortion.hpp"
#include "argus/perception/geometry/include/static_camera_projector.hpp"
using namespace follow;
int main(int argc, char** argv) {
    assert(argc == 4);
    auto profile = read_native_json(argv[1]);
    auto g = profile.at("geometry");
    auto m = read_native_json(argv[2]);
    m["ground_center_radius_m"] = profile["measurement_overrides"]["ground_center_radius_m"];
    const auto d = read_pixel_distortion(g);
    for (double x : {-.6, -.3, 0., .3, .6}) {
        for (double y : {-.45, 0., .45}) {
            auto raw = distort_normalized(x, y, d);
            auto ray = undistort_normalized(raw[0], raw[1], d);
            assert(std::hypot(ray[0] - x, ray[1] - y) < 1e-9);
        }
    }
    // Independent OpenCV 4.8 undistortPointsIter reference (100 iterations, eps 1e-14).
    const std::array<std::array<double, 4>, 3> references{
        {{30., 30., -.6134978542253395, -.4496531558638641},
         {610., 450., .5562360680006183, .3966739932843803},
         {320., 240., -.0222533620142885, -.0214135933677228}}};
    for (const auto& ref : references) {
        auto ray =
            undistort_normalized((ref[0] - 331.194) / 502.949, (ref[1] - 250.783) / 503.501, d);
        assert(std::hypot(ray[0] - ref[2], ray[1] - ref[3]) < 1e-9);
    }
    auto invalid = g;
    invalid["input_pixels"] = "rectified";
    bool rejected = false;
    try {
        (void)read_pixel_distortion(invalid);
    } catch (...) {
        rejected = true;
    }
    assert(rejected);
    const std::string uid = profile.at("expected_uid");
    const long long stamp = 1000000000;
    for (double pitch : {0., .25, -.25}) {
        Eigen::Matrix3d mount;
        for (int i = 0; i < 9; ++i) {
            mount(i / 3, i % 3) = g["camera_to_body"][i / 3][i % 3];
        }
        Eigen::Matrix3d body =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
        double ground = (body * Eigen::Vector3d(.105, 0, 0))[2] + 2 * body(2, 2);
        Eigen::Vector3d target(2., .3, ground);
        Eigen::Vector3d optical =
            mount.transpose() * body.transpose() * (target - body * Eigen::Vector3d(.125, 0, 0));
        auto raw = distort_normalized(optical[0] / optical[2], optical[1] / optical[2], d);
        double u = g["cx"].get<double>() + g["fx"].get<double>() * raw[0];
        double v = g["cy"].get<double>() + g["fy"].get<double>() * raw[1];
        NativeJson pixels{{"schema", "argus.perception.observations.v2"},
                          {"clock_domain", "host_monotonic"},
                          {"coordinate_frame", "camera_pixel"},
                          {"captured_monotonic_ns", stamp},
                          {"image_width", 640},
                          {"image_height", 480},
                          {"detections",
                           {{{"x1", u - 12},
                             {"x2", u + 12},
                             {"y1", v - 24},
                             {"y2", v},
                             {"confidence", .9},
                             {"class_id", 2},
                             {"label", "tank"}}}}};
        NativeJson history;
        history["ATTITUDE"] = {{{"received_monotonic_ns", stamp},
                                {"data", {{"roll", 0.}, {"pitch", pitch}, {"yaw", 0.}}}}};
        history["LOCAL_POSITION_NED"] = {
            {{"received_monotonic_ns", stamp}, {"data", {{"x", 0.}, {"y", 0.}, {"z", 0.}}}}};
        history["DISTANCE_SENSOR"] = {{{"received_monotonic_ns", stamp},
                                       {"data",
                                        {{"orientation", 25},
                                         {"current_distance", 200},
                                         {"min_distance", 10},
                                         {"max_distance", 1000}}}}};
        NativeJson fc{{"schema", "argus.fc.observations.v1"},
                      {"clock_domain", "host_monotonic"},
                      {"epoch", 1},
                      {"state",
                       {{"uid", uid},
                        {"identity_ok", true},
                        {"fresh", true},
                        {"local_ok", true},
                        {"flow_quality", 200}}},
                      {"sensors", {{"flow_valid", true}}},
                      {"history", history}};
        PiGroundProjector projector(g, m, uid);
        auto projected = projector.project(pixels, fc, stamp + 1000000);
        assert(projected.detections.size() == 1);
        Eigen::Vector2d expected(2., .3);
        expected += expected.normalized() * m["ground_center_radius_m"].get<double>();
        assert((projected.detections[0].metric.point - expected).norm() < 1e-7);
        for (int fault = 0; fault < 5; ++fault) {
            auto bad = fc;
            auto pix = pixels;
            if (fault == 0) {
                bad["state"]["uid"] = "wrong";
            }
            if (fault == 1) {
                bad["history"]["ATTITUDE"][0]["received_monotonic_ns"] = stamp - 51000000;
            }
            if (fault == 2) {
                bad["history"]["DISTANCE_SENSOR"][0]["data"]["orientation"] = 0;
            }
            if (fault == 3) {
                pix["image_width"] = 800;
            }
            if (fault == 4) {
                bad["sensors"]["flow_valid"] = false;
            }
            rejected = false;
            try {
                (void)projector.project(pix, bad, stamp + 1000000);
            } catch (...) {
                rejected = true;
            }
            assert(rejected);
        }
        rejected = false;
        try {
            (void)projector.project(pixels, fc, stamp + 250000000);
        } catch (...) {
            rejected = true;
        }
        assert(rejected);
        // FC-free association remains possible with the same RGB optical model.
        StaticCameraProjector static_projector(argv[3], 640, 480, g);
        auto camera_frame = static_projector.project(pixels, stamp + 1000000);
        assert(!camera_frame.detections.empty());
    }
    std::cout
        << "RGB distortion, range lever arm, target radius, camera-only and safety gates passed\n";
}
