// Raw OPENCV pixels <-> ideal pinhole rays. No image resampling or FC access.
#pragma once
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace follow {
using Distortion4 = std::array<double, 4>;
inline Distortion4 read_pixel_distortion(const nlohmann::json& config) {
    const auto d = config.value("distortion", Distortion4{});
    if (config.contains("distortion") &&
        (config.value("distortion_model", "") != "opencv_pinhole_4coeff" ||
         config.value("input_pixels", "") != "raw")) {
        throw std::invalid_argument("distortion requires explicit raw OPENCV pixels");
    }
    for (double v : d) {
        if (!std::isfinite(v)) {
            throw std::invalid_argument("invalid distortion");
        }
    }
    return d;
}
inline std::array<double, 2> distort_normalized(double x, double y, const Distortion4& d) {
    const double r2 = x * x + y * y;
    const double radial = 1.0 + d[0] * r2 + d[1] * r2 * r2;
    return {x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x),
            y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y};
}
inline std::array<double, 2> undistort_normalized(double xd, double yd, const Distortion4& d) {
    for (double v : d) {
        if (!std::isfinite(v)) {
            throw std::invalid_argument("nonfinite OPENCV distortion");
        }
    }
    if (!std::isfinite(xd) || !std::isfinite(yd)) {
        throw std::runtime_error("nonfinite camera ray");
    }
    double x = xd, y = yd;
    for (int i = 0; i < 30; ++i) {
        const auto p = distort_normalized(x, y, d);
        const double ex = p[0] - xd, ey = p[1] - yd;
        if (std::hypot(ex, ey) < 1e-12) {
            return {x, y};
        }
        constexpr double h = 1e-6;
        const auto px = distort_normalized(x + h, y, d);
        const auto py = distort_normalized(x, y + h, d);
        const double a = (px[0] - p[0]) / h, b = (py[0] - p[0]) / h;
        const double c = (px[1] - p[1]) / h, e = (py[1] - p[1]) / h;
        const double det = a * e - b * c;
        if (!std::isfinite(det) || det <= 1e-8) {
            break;
        }
        x -= (e * ex - b * ey) / det;
        y -= (a * ey - c * ex) / det;
        if (!std::isfinite(x) || !std::isfinite(y) || std::hypot(x, y) > 10.0) {
            break;
        }
    }
    throw std::runtime_error("OPENCV inverse distortion did not converge");
}
}  // namespace follow
