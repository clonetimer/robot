#include "robot_id/dynamic_parameters.hpp"
#include "robot_id/config.hpp"

#include <cmath>
#include <stdexcept>

namespace robot_id {

Eigen::VectorXd makeNominalMismatchedParameters(const Eigen::VectorXd& theta_true) {
    if (theta_true.size() % config::kParametersPerLink != 0 || !theta_true.allFinite()) {
        throw std::runtime_error("Dynamic parameter vector must contain 10 finite parameters per moving link.");
    }

    Eigen::VectorXd theta = theta_true;
    // Deterministic 15-25% model mismatch, applied in physically meaningful coordinates.
    // Dynamic parameters store inertia about the link-frame origin. Recover (m, c, I_C),
    // perturb mass and COM inertia, then reconstruct I_O with the parallel-axis theorem.
    // This avoids creating a nominal model with negative/invalid COM inertia.
    constexpr double mass_scale[] = {0.82, 1.18, 0.85, 1.20, 0.80, 1.15};
    constexpr double com_inertia_scale[] = {1.22, 0.82, 1.18, 0.80, 1.25, 0.84};
    const Eigen::Index links = theta.size() / config::kParametersPerLink;
    if (links != config::kDof) {
        throw std::runtime_error("MVP nominal mismatch is defined for the frozen 6R project model.");
    }

    for (Eigen::Index link = 0; link < links; ++link) {
        const Eigen::Index base = config::kParametersPerLink * link;
        const double mass = theta_true(base);
        if (!std::isfinite(mass) || mass <= 0.0) {
            throw std::runtime_error("Nominal mismatch requires strictly positive link masses.");
        }

        const Eigen::Vector3d first_moment = theta_true.segment<3>(base + 1);
        const Eigen::Vector3d com = first_moment / mass;

        Eigen::Matrix3d inertia_origin;
        inertia_origin << theta_true(base + 4), theta_true(base + 5), theta_true(base + 7),
                          theta_true(base + 5), theta_true(base + 6), theta_true(base + 8),
                          theta_true(base + 7), theta_true(base + 8), theta_true(base + 9);
        const Eigen::Matrix3d parallel_axis =
            com.squaredNorm() * Eigen::Matrix3d::Identity() - com * com.transpose();
        const Eigen::Matrix3d inertia_com = inertia_origin - mass * parallel_axis;

        const double mass_nominal = mass_scale[link] * mass;
        const Eigen::Matrix3d inertia_com_nominal = com_inertia_scale[link] * inertia_com;
        const Eigen::Matrix3d inertia_origin_nominal =
            inertia_com_nominal + mass_nominal * parallel_axis;

        theta(base) = mass_nominal;
        theta.segment<3>(base + 1) = mass_nominal * com;
        theta(base + 4) = inertia_origin_nominal(0, 0);
        theta(base + 5) = inertia_origin_nominal(0, 1);
        theta(base + 6) = inertia_origin_nominal(1, 1);
        theta(base + 7) = inertia_origin_nominal(0, 2);
        theta(base + 8) = inertia_origin_nominal(1, 2);
        theta(base + 9) = inertia_origin_nominal(2, 2);
    }
    if (!theta.allFinite()) {
        throw std::runtime_error("Nominal mismatch produced NaN/Inf dynamic parameters.");
    }
    return theta;
}

}  // namespace robot_id
