#include "robot_id/config.hpp"
#include "robot_id/identification.hpp"
#include "robot_id/robot_model.hpp"
#include "robot_id/trajectory.hpp"

#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected URDF path.\n";
        return 2;
    }

    robot_id::RobotModel robot(argv[1]);
    const auto id_traj = robot_id::makeIdentificationTrajectory();
    constexpr double kPi = 3.14159265358979323846;
    const double duration = 2.0 * 2.0 * kPi / id_traj.fundamental();

    // Noise-free integration test: truncated-SVD should recover an equivalent
    // dynamic model even though the full 60-vector is structurally redundant.
    const auto dataset = robot_id::generateSyntheticDataset(
        robot, id_traj, duration, 0.02, 0.0, robot_id::config::kRandomSeed);
    const auto id = robot_id::identifyByScaledSvd(
        dataset, robot_id::config::kSvdRelativeRankThreshold);
    const auto val = robot_id::validateTorquePrediction(
        robot, robot_id::makeValidationTrajectory(), id.theta_hat, 8.0, 0.02);

    std::cout << "rank=" << id.numerical_rank
              << ", train RMSE=" << id.train_rmse_nm
              << " Nm, validation NRMSE=" << val.overall_nrmse << '\n';

    return (id.numerical_rank > 0 && id.numerical_rank < robot.parameterCount() &&
            id.train_rmse_nm < 1e-8 && val.overall_nrmse < 1e-7) ? 0 : 1;
}
