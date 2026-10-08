#include "robot_id/identification.hpp"
#include "robot_id/random.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_id {

RegressionDataset generateSyntheticDataset(const RobotModel& robot,
                                            const MultiSineTrajectory& trajectory,
                                            double duration_s,
                                            double sample_period_s,
                                            double torque_noise_std_nm,
                                            std::uint32_t random_seed) {
    if (robot.dof() != trajectory.dof() || !std::isfinite(duration_s) || duration_s <= 0.0 ||
        !std::isfinite(sample_period_s) || sample_period_s <= 0.0 ||
        !std::isfinite(torque_noise_std_nm) || torque_noise_std_nm < 0.0) {
        throw std::runtime_error("Invalid dataset generation arguments.");
    }

    const std::size_t samples = static_cast<std::size_t>(std::floor(duration_s / sample_period_s)) + 1;
    const int dof = robot.dof();
    const int p = robot.parameterCount();

    RegressionDataset dataset;
    dataset.samples = samples;
    dataset.W.resize(static_cast<Eigen::Index>(samples) * dof, p);
    dataset.tau.resize(static_cast<Eigen::Index>(samples) * dof);

    DeterministicNormalGenerator noise(random_seed);

    for (std::size_t i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) * sample_period_s;
        const auto x = trajectory.sample(t);
        const Eigen::MatrixXd Y = robot.torqueRegressor(x.q, x.v, x.a);
        Eigen::VectorXd tau = robot.inverseDynamics(x.q, x.v, x.a);

        for (int j = 0; j < dof; ++j) {
            tau(j) += noise.sample(0.0, torque_noise_std_nm);
        }

        const Eigen::Index row = static_cast<Eigen::Index>(i) * dof;
        dataset.W.middleRows(row, dof) = Y;
        dataset.tau.segment(row, dof) = tau;
    }
    return dataset;
}

TorqueValidationMetrics validateTorquePrediction(const RobotModel& robot,
                                                  const MultiSineTrajectory& trajectory,
                                                  const Eigen::VectorXd& theta,
                                                  double duration_s,
                                                  double sample_period_s) {
    if (theta.size() != robot.parameterCount() || !theta.allFinite() ||
        robot.dof() != trajectory.dof() || !std::isfinite(duration_s) || duration_s <= 0.0 ||
        !std::isfinite(sample_period_s) || sample_period_s <= 0.0) {
        throw std::runtime_error("Invalid torque-validation arguments.");
    }

    const int dof = robot.dof();
    const std::size_t samples = static_cast<std::size_t>(std::floor(duration_s / sample_period_s)) + 1;
    Eigen::VectorXd err_sq = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd truth_sq = Eigen::VectorXd::Zero(dof);
    double total_err_sq = 0.0;
    double total_truth_sq = 0.0;

    for (std::size_t i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) * sample_period_s;
        const auto x = trajectory.sample(t);
        const Eigen::MatrixXd Y = robot.torqueRegressor(x.q, x.v, x.a);
        const Eigen::VectorXd tau_true = robot.inverseDynamics(x.q, x.v, x.a);
        const Eigen::VectorXd tau_pred = Y * theta;
        const Eigen::VectorXd e = tau_pred - tau_true;
        err_sq += e.array().square().matrix();
        truth_sq += tau_true.array().square().matrix();
        total_err_sq += e.squaredNorm();
        total_truth_sq += tau_true.squaredNorm();
    }

    TorqueValidationMetrics metrics;
    metrics.rmse_nm = (err_sq / static_cast<double>(samples)).array().sqrt().matrix();
    metrics.nrmse.resize(dof);
    for (int j = 0; j < dof; ++j) {
        const double truth_rms = std::sqrt(truth_sq(j) / static_cast<double>(samples));
        metrics.nrmse(j) = metrics.rmse_nm(j) / std::max(truth_rms, 1e-9);
    }
    metrics.overall_rmse_nm = std::sqrt(total_err_sq / static_cast<double>(samples * dof));
    metrics.overall_nrmse = metrics.overall_rmse_nm /
        std::max(std::sqrt(total_truth_sq / static_cast<double>(samples * dof)), 1e-9);
    return metrics;
}

}  // namespace robot_id
