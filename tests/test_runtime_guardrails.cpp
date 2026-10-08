#include "robot_id/experiment.hpp"
#include "robot_id/identification.hpp"
#include "robot_id/robot_model.hpp"
#include "robot_id/trajectory.hpp"

#include <Eigen/Core>

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
bool expectThrow(const std::string& name, const std::function<void()>& fn) {
    try {
        fn();
        std::cerr << "Expected exception was not thrown: " << name << '\n';
        return false;
    } catch (const std::exception&) {
        return true;
    }
}
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected URDF path.\n";
        return 2;
    }

    robot_id::RobotModel robot(argv[1]);
    const int n = robot.dof();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
    bool ok = true;

    Eigen::VectorXd short_q = Eigen::VectorXd::Zero(n - 1);
    ok &= expectThrow("inverseDynamics wrong q size", [&] { robot.inverseDynamics(short_q, v, a); });
    ok &= expectThrow("torqueRegressor wrong v size", [&] { robot.torqueRegressor(q, short_q, a); });
    ok &= expectThrow("forwardDynamics wrong tau size", [&] { robot.forwardDynamics(q, v, short_q); });
    ok &= expectThrow("integrate wrong delta size", [&] { robot.integrate(q, short_q); });

    q(0) = std::numeric_limits<double>::quiet_NaN();
    ok &= expectThrow("inverseDynamics NaN", [&] { robot.inverseDynamics(q, v, a); });
    q.setZero();

    const auto trajectory = robot_id::makeIdentificationTrajectory();
    ok &= expectThrow("trajectory non-finite time", [&] {
        trajectory.sample(std::numeric_limits<double>::infinity());
    });
    ok &= expectThrow("dataset zero duration", [&] {
        robot_id::generateSyntheticDataset(robot, trajectory, 0.0, 0.01, 0.2, 1U);
    });
    ok &= expectThrow("dataset negative noise", [&] {
        robot_id::generateSyntheticDataset(robot, trajectory, 1.0, 0.01, -0.1, 1U);
    });
    ok &= expectThrow("validation wrong theta", [&] {
        robot_id::validateTorquePrediction(robot, trajectory, Eigen::VectorXd::Zero(1), 1.0, 0.01);
    });

    const Eigen::VectorXd theta = robot.dynamicParameters();
    ok &= expectThrow("control zero step", [&] {
        robot_id::simulateTracking(robot, robot_id::makeControlTrajectory(),
            robot_id::ControllerKind::ComputedTorqueIdentified,
            theta, theta, 1.0, 0.0, 10);
    });
    ok &= expectThrow("control invalid log stride", [&] {
        robot_id::simulateTracking(robot, robot_id::makeControlTrajectory(),
            robot_id::ControllerKind::ComputedTorqueIdentified,
            theta, theta, 1.0, 0.001, 0);
    });

    robot_id::ControlRun invalid_run;
    invalid_run.metrics.kind = robot_id::ControllerKind::PdOnly;
    invalid_run.metrics.joint_rmse_rad = Eigen::VectorXd::Zero(n);
    invalid_run.metrics.overall_rmse_rad = 0.0;
    invalid_run.metrics.max_abs_error_rad = 0.0;
    invalid_run.metrics.saturation_fraction = 0.0;
    invalid_run.metrics.peak_unsaturated_effort_ratio = 0.0;
    invalid_run.metrics.min_position_limit_margin_rad = 1.0;
    invalid_run.metrics.peak_velocity_limit_ratio = std::numeric_limits<double>::quiet_NaN();
    ok &= expectThrow("invalid control CSV metric", [&] {
        robot_id::writeControlMetricsCsv("runtime_guardrails_should_not_exist.csv", {invalid_run});
    });

    std::cout << "runtime guardrails " << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}
