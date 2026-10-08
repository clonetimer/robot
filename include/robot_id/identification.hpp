#pragma once

#include "robot_id/least_squares.hpp"
#include "robot_id/robot_model.hpp"
#include "robot_id/trajectory.hpp"

#include <Eigen/Core>

#include <cstdint>

namespace robot_id {

RegressionDataset generateSyntheticDataset(const RobotModel& robot,
                                            const MultiSineTrajectory& trajectory,
                                            double duration_s,
                                            double sample_period_s,
                                            double torque_noise_std_nm,
                                            std::uint32_t random_seed);

struct TorqueValidationMetrics {
    Eigen::VectorXd rmse_nm;
    Eigen::VectorXd nrmse;
    double overall_rmse_nm{};
    double overall_nrmse{};
};

TorqueValidationMetrics validateTorquePrediction(const RobotModel& robot,
                                                  const MultiSineTrajectory& trajectory,
                                                  const Eigen::VectorXd& theta,
                                                  double duration_s,
                                                  double sample_period_s);

}  // namespace robot_id
