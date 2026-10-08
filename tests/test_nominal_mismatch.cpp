#include "robot_id/config.hpp"
#include "robot_id/dynamic_parameters.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {

Eigen::VectorXd makeFrozenTruthParameters() {
    constexpr std::array<double, 6> kMass = {4.5, 6.0, 4.0, 3.0, 2.0, 1.5};
    constexpr std::array<double, 6> kComX = {0.0, 0.16, 0.14, 0.05, 0.04, 0.03};
    constexpr std::array<double, 6> kComZ = {0.06, 0.0, 0.0, 0.0, 0.0, 0.0};
    constexpr std::array<std::array<double, 3>, 6> kComInertiaDiag = {{
        {{0.00915, 0.00915, 0.00750}},
        {{0.00640, 0.05440, 0.05440}},
        {{0.0032666667, 0.0277666667, 0.0277666667}},
        {{0.100, 0.150, 0.150}},
        {{0.060, 0.080, 0.080}},
        {{0.040, 0.050, 0.050}},
    }};

    Eigen::VectorXd theta(robot_id::config::kFullParameterCount);
    for (int link = 0; link < robot_id::config::kDof; ++link) {
        const double m = kMass.at(link);
        Eigen::Vector3d c(kComX.at(link), 0.0, kComZ.at(link));
        Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Zero();
        inertia_com.diagonal() = Eigen::Vector3d(
            kComInertiaDiag.at(link).at(0),
            kComInertiaDiag.at(link).at(1),
            kComInertiaDiag.at(link).at(2));
        const Eigen::Matrix3d parallel_axis =
            c.squaredNorm() * Eigen::Matrix3d::Identity() - c * c.transpose();
        const Eigen::Matrix3d inertia_origin = inertia_com + m * parallel_axis;

        const int base = robot_id::config::kParametersPerLink * link;
        theta(base) = m;
        theta.segment<3>(base + 1) = m * c;
        theta(base + 4) = inertia_origin(0, 0);
        theta(base + 5) = inertia_origin(0, 1);
        theta(base + 6) = inertia_origin(1, 1);
        theta(base + 7) = inertia_origin(0, 2);
        theta(base + 8) = inertia_origin(1, 2);
        theta(base + 9) = inertia_origin(2, 2);
    }
    return theta;
}

}  // namespace

int main() {
    const Eigen::VectorXd truth = makeFrozenTruthParameters();
    const Eigen::VectorXd nominal = robot_id::makeNominalMismatchedParameters(truth);

    double min_mass = 1e9;
    double min_inertia_eigenvalue = 1e9;
    double max_com_change = 0.0;
    bool triangle_ok = true;

    for (int link = 0; link < robot_id::config::kDof; ++link) {
        const int base = robot_id::config::kParametersPerLink * link;
        const double m_true = truth(base);
        const double m = nominal(base);
        const Eigen::Vector3d c_true = truth.segment<3>(base + 1) / m_true;
        const Eigen::Vector3d c = nominal.segment<3>(base + 1) / m;
        max_com_change = std::max(max_com_change, (c - c_true).norm());
        min_mass = std::min(min_mass, m);

        Eigen::Matrix3d inertia_origin;
        inertia_origin << nominal(base + 4), nominal(base + 5), nominal(base + 7),
                          nominal(base + 5), nominal(base + 6), nominal(base + 8),
                          nominal(base + 7), nominal(base + 8), nominal(base + 9);
        const Eigen::Matrix3d parallel_axis =
            c.squaredNorm() * Eigen::Matrix3d::Identity() - c * c.transpose();
        const Eigen::Matrix3d inertia_com = inertia_origin - m * parallel_axis;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(inertia_com);
        if (solver.info() != Eigen::Success) {
            return 1;
        }
        min_inertia_eigenvalue = std::min(min_inertia_eigenvalue, solver.eigenvalues().minCoeff());
        const Eigen::Vector3d d = inertia_com.diagonal();
        triangle_ok = triangle_ok &&
            (d(0) + d(1) >= d(2) - 1e-12) &&
            (d(0) + d(2) >= d(1) - 1e-12) &&
            (d(1) + d(2) >= d(0) - 1e-12);
    }

    std::cout << "min mass=" << min_mass
              << ", min COM-inertia eigenvalue=" << min_inertia_eigenvalue
              << ", max COM change=" << max_com_change << '\n';
    return (min_mass > 0.0 && min_inertia_eigenvalue > 0.0 &&
            max_com_change < 1e-14 && triangle_ok) ? 0 : 1;
}
