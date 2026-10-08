#include "robot_id/trajectory.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot_id {
namespace {
constexpr double kPi = 3.14159265358979323846;

MultiSineTrajectory makeTrajectory(const Eigen::VectorXd& offset,
                                   const Eigen::VectorXd& joint_scale,
                                   int harmonics,
                                   double fundamental,
                                   double phase_bias,
                                   double harmonic_decay_power) {
    const int dof = static_cast<int>(offset.size());
    Eigen::MatrixXd amplitudes(dof, harmonics);
    Eigen::MatrixXd phases(dof, harmonics);

    for (int j = 0; j < dof; ++j) {
        for (int k = 1; k <= harmonics; ++k) {
            amplitudes(j, k - 1) = joint_scale(j) / std::pow(static_cast<double>(k), harmonic_decay_power);
            double phase = phase_bias + 0.71 * j + 0.43 * k + 0.13 * j * k;
            phases(j, k - 1) = std::fmod(phase, 2.0 * kPi);
        }
    }
    return MultiSineTrajectory(offset, amplitudes, phases, fundamental);
}
}  // namespace

MultiSineTrajectory::MultiSineTrajectory(Eigen::VectorXd offset,
                                         Eigen::MatrixXd amplitudes,
                                         Eigen::MatrixXd phases,
                                         double fundamental_rad_s)
    : offset_(std::move(offset)),
      amplitudes_(std::move(amplitudes)),
      phases_(std::move(phases)),
      fundamental_rad_s_(fundamental_rad_s) {
    if (amplitudes_.rows() != offset_.size() || phases_.rows() != offset_.size() ||
        amplitudes_.cols() != phases_.cols() || !offset_.allFinite() ||
        !amplitudes_.allFinite() || !phases_.allFinite() ||
        !std::isfinite(fundamental_rad_s_) || fundamental_rad_s_ <= 0.0) {
        throw std::runtime_error("Invalid multi-sine trajectory dimensions or fundamental frequency.");
    }
}

TrajectoryState MultiSineTrajectory::sample(double t) const {
    if (!std::isfinite(t)) {
        throw std::runtime_error("Trajectory sample time must be finite.");
    }
    TrajectoryState x;
    x.q = offset_;
    x.v = Eigen::VectorXd::Zero(offset_.size());
    x.a = Eigen::VectorXd::Zero(offset_.size());

    for (Eigen::Index j = 0; j < offset_.size(); ++j) {
        for (Eigen::Index h = 0; h < amplitudes_.cols(); ++h) {
            const double k = static_cast<double>(h + 1);
            const double omega = k * fundamental_rad_s_;
            const double phase = omega * t + phases_(j, h);
            const double amp = amplitudes_(j, h);
            x.q(j) += amp * std::sin(phase);
            x.v(j) += amp * omega * std::cos(phase);
            x.a(j) -= amp * omega * omega * std::sin(phase);
        }
    }
    return x;
}

MultiSineTrajectory makeIdentificationTrajectory() {
    Eigen::VectorXd offset(6);
    offset << 0.0, -0.50, 0.70, 0.0, 0.40, 0.0;
    Eigen::VectorXd scale(6);
    scale << 0.55, 0.42, 0.42, 0.55, 0.48, 0.55;
    return makeTrajectory(offset, scale, 5, 0.70, 0.17, 2.0);
}

MultiSineTrajectory makeValidationTrajectory() {
    Eigen::VectorXd offset(6);
    offset << 0.15, -0.35, 0.50, -0.10, 0.25, 0.10;
    Eigen::VectorXd scale(6);
    scale << 0.34, 0.30, 0.30, 0.36, 0.32, 0.36;
    return makeTrajectory(offset, scale, 3, 0.53, 1.11, 2.0);
}

MultiSineTrajectory makeControlTrajectory() {
    Eigen::VectorXd offset(6);
    offset << 0.10, -0.45, 0.60, 0.05, 0.30, -0.05;
    Eigen::VectorXd scale(6);
    scale << 0.22, 0.18, 0.18, 0.24, 0.20, 0.22;
    return makeTrajectory(offset, scale, 2, 0.42, 2.07, 2.0);
}

}  // namespace robot_id
