#include "robot_id/config.hpp"
#include "robot_id/trajectory.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    const std::vector<robot_id::MultiSineTrajectory> trajectories = {
        robot_id::makeIdentificationTrajectory(),
        robot_id::makeValidationTrajectory(),
        robot_id::makeControlTrajectory(),
    };

    double q_max = 0.0, v_max = 0.0, a_max = 0.0;
    for (const auto& traj : trajectories) {
        for (int i = 0; i <= 5000; ++i) {
            const auto x = traj.sample(0.01 * i);
            q_max = std::max(q_max, x.q.cwiseAbs().maxCoeff());
            v_max = std::max(v_max, x.v.cwiseAbs().maxCoeff());
            a_max = std::max(a_max, x.a.cwiseAbs().maxCoeff());
        }
    }

    std::cout << "max |q|=" << q_max << ", |v|=" << v_max << ", |a|=" << a_max << '\n';
    return (q_max < robot_id::config::kTrajectoryMaxAbsPositionRad &&
            v_max < robot_id::config::kTrajectoryMaxAbsVelocityRadS &&
            a_max < robot_id::config::kTrajectoryMaxAbsAccelerationRadS2) ? 0 : 1;
}
