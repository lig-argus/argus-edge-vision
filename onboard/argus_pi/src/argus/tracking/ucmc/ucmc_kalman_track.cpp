#include "argus/tracking/ucmc/include/ucmc_kalman_track.hpp"

#include <Eigen/LU>
#include <cmath>
#include <stdexcept>
namespace follow {
FloatTrackIdSequence UcmcKalmanTrack::next_id_;
namespace {
Eigen::Matrix<double, 2, 4> observation_matrix() {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = H(1, 2) = 1;
    return H;
}
}  // namespace
UcmcKalmanTrack::UcmcKalmanTrack(const MetricBoxObservation& d, double wx, double wy, double vmax,
                                 double dt)
    : id(next_id_.allocate()) {
    if (id <= 0) {
        throw std::overflow_error("UCMC track ID exhausted");
    }
    x[0] = d.point[0];
    x[2] = d.point[1];
    P.diagonal() << 1, vmax * vmax / 3., 1, vmax * vmax / 3.;
    width = d.box[2] - d.box[0];
    height = d.box[3] - d.box[1];
    set_dt(dt, wx, wy);
}
void UcmcKalmanTrack::set_dt(double dt, double wx, double wy) {
    if (!std::isfinite(dt) || dt < 0) {
        throw std::invalid_argument("invalid UCMC dt");
    }
    F.setIdentity();
    F(0, 1) = F(2, 3) = dt;
    Eigen::Matrix<double, 4, 2> G = Eigen::Matrix<double, 4, 2>::Zero();
    G(0, 0) = G(2, 1) = .5 * dt * dt;
    G(1, 0) = G(3, 1) = dt;
    Eigen::Matrix2d noise = Eigen::Matrix2d::Zero();
    noise.diagonal() << wx, wy;
    Q = G * noise * G.transpose();
}
void UcmcKalmanTrack::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
    ++age;
}
void UcmcKalmanTrack::update(const MetricBoxObservation& d) {
    auto H = observation_matrix();
    Eigen::Matrix2d S = H * P * H.transpose() + d.covariance;
    if (!S.allFinite() || S.determinant() <= 0) {
        throw std::runtime_error("invalid UCMC innovation covariance");
    }
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
    x += K * (d.point - H * x);
    // FilterPy와 동일한 Joseph covariance update를 쓴다.
    Eigen::Matrix4d I_KH = Eigen::Matrix4d::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * d.covariance * K.transpose();
}
double UcmcKalmanTrack::distance(const MetricBoxObservation& d) const {
    auto H = observation_matrix();
    Eigen::Vector2d residual = d.point - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + d.covariance;
    double determinant = S.determinant();
    if (!S.allFinite() || determinant <= 0) {
        throw std::runtime_error("invalid UCMC association covariance");
    }
    double value = residual.dot(S.inverse() * residual) + std::log(determinant);
    if (!std::isfinite(value)) {
        throw std::runtime_error("invalid UCMC association cost");
    }
    return value;
}
}  // namespace follow
