#include "robot_id/config.hpp"
#include "robot_id/random.hpp"
#include "robot_id/robot_model.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected URDF path.\n";
        return 2;
    }

    robot_id::RobotModel robot(argv[1]);
    robot_id::DeterministicNormalGenerator rng(robot_id::config::kA0RandomSeed);

    double worst = 0.0;
    for (int sample = 0; sample < robot_id::config::kA0StateSamples; ++sample) {
        Eigen::VectorXd q(robot.dof()), v(robot.dof()), a(robot.dof());
        for (int j = 0; j < robot.dof(); ++j) {
            q(j) = (2.0 * rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsPositionRad;
            v(j) = (2.0 * rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsVelocityRadS;
            a(j) = (2.0 * rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsAccelerationRadS2;
        }
        const Eigen::VectorXd tau_rnea = robot.inverseDynamics(q, v, a);
        const Eigen::VectorXd tau_reg = robot.torqueRegressor(q, v, a) * robot.dynamicParameters();
        const double rel = (tau_rnea - tau_reg).norm() / std::max(tau_rnea.norm(), 1e-12);
        worst = std::max(worst, rel);
    }

    std::cout << "states=" << robot_id::config::kA0StateSamples
              << ", worst relative error=" << worst << '\n';
    return worst < robot_id::config::kA0MaxRegressorRelativeError ? 0 : 1;
}
