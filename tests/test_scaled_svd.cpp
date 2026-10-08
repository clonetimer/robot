#include "robot_id/least_squares.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>

int main() {
    constexpr int rows = 120;
    constexpr int cols = 5;
    robot_id::RegressionDataset dataset;
    dataset.W.resize(rows, cols);
    dataset.tau.resize(rows);
    dataset.samples = rows;

    for (int i = 0; i < rows; ++i) {
        const double x = -1.0 + 2.0 * static_cast<double>(i) / static_cast<double>(rows - 1);
        const double c0 = 1.0;
        const double c1 = x;
        const double c2 = x * x;
        dataset.W(i, 0) = c0;
        dataset.W(i, 1) = c1;
        dataset.W(i, 2) = c2;
        dataset.W(i, 3) = c0 + 2.0 * c1;
        dataset.W(i, 4) = -c1 + c2;
    }

    Eigen::VectorXd theta_true(cols);
    theta_true << 2.0, -1.5, 0.7, 0.4, -0.2;
    dataset.tau = dataset.W * theta_true;

    const auto result = robot_id::identifyByScaledSvd(dataset, 1e-10);
    const Eigen::VectorXd residual = dataset.W * result.theta_hat - dataset.tau;
    const double relative_residual = residual.norm() / std::max(dataset.tau.norm(), 1e-12);

    std::cout << "rank=" << result.numerical_rank
              << ", relative residual=" << relative_residual << '\n';

    // W has five columns but only three independent column directions.
    return (result.numerical_rank == 3 && relative_residual < 1e-11) ? 0 : 1;
}
