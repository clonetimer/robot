#include "robot_id/robot_model.hpp"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <iostream>

namespace {
bool near(double a, double b, double tol = 1e-11) {
    return std::abs(a - b) <= tol;
}
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected URDF path.\n";
        return 2;
    }

    robot_id::RobotModel robot(argv[1]);
    if (robot.dof() != 6 || robot.parameterCount() != 60) {
        std::cerr << "Unexpected model dimensions.\n";
        return 1;
    }

    const std::array<double, 6> lower = {-2.8, -2.6, -2.6, -3.0, -2.8, -3.0};
    const std::array<double, 6> upper = { 2.8,  2.6,  2.6,  3.0,  2.8,  3.0};
    const std::array<double, 6> effort = {160.0, 160.0, 120.0, 70.0, 45.0, 30.0};
    const std::array<double, 6> velocity = {2.0, 2.0, 2.2, 2.8, 3.0, 3.2};

    for (int i = 0; i < 6; ++i) {
        if (!near(robot.lowerPositionLimits()(i), lower[static_cast<std::size_t>(i)]) ||
            !near(robot.upperPositionLimits()(i), upper[static_cast<std::size_t>(i)]) ||
            !near(robot.effortLimits()(i), effort[static_cast<std::size_t>(i)]) ||
            !near(robot.velocityLimits()(i), velocity[static_cast<std::size_t>(i)])) {
            std::cerr << "URDF limit contract mismatch at joint " << (i + 1) << ".\n";
            return 1;
        }
    }

    // Golden dynamic parameters analytically converted from the frozen URDF's
    // COM-frame inertias using the parallel-axis theorem. This independently
    // guards the tinyxml2 -> Pinocchio model-construction compatibility layer.
    Eigen::VectorXd expected = Eigen::VectorXd::Zero(60);
    auto set_link = [&](int link, double m, double mcx, double mcy, double mcz,
                        double ixx, double ixy, double iyy, double ixz, double iyz, double izz) {
        const int k = 10 * (link - 1);
        expected.segment<10>(k) << m, mcx, mcy, mcz, ixx, ixy, iyy, ixz, iyz, izz;
    };
    set_link(1, 4.5, 0.0, 0.0, 0.27, 0.02535, 0.0, 0.02535, 0.0, 0.0, 0.00750);
    set_link(2, 6.0, 0.96, 0.0, 0.0, 0.00640, 0.0, 0.20800, 0.0, 0.0, 0.20800);
    set_link(3, 4.0, 0.56, 0.0, 0.0, 0.0032666667, 0.0, 0.1061666667, 0.0, 0.0, 0.1061666667);
    set_link(4, 3.0, 0.15, 0.0, 0.0, 0.10000, 0.0, 0.15750, 0.0, 0.0, 0.15750);
    set_link(5, 2.0, 0.08, 0.0, 0.0, 0.06000, 0.0, 0.08320, 0.0, 0.0, 0.08320);
    set_link(6, 1.5, 0.045, 0.0, 0.0, 0.04000, 0.0, 0.05135, 0.0, 0.0, 0.05135);

    const double max_dynamic_parameter_error =
        (robot.dynamicParameters() - expected).cwiseAbs().maxCoeff();
    std::cout << "max dynamic-parameter loader error=" << max_dynamic_parameter_error << '\n';
    return max_dynamic_parameter_error < 1e-10 ? 0 : 1;
}
