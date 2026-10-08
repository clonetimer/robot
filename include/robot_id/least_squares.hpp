#pragma once

#include <Eigen/Core>

#include <cstddef>

namespace robot_id {

struct RegressionDataset {
    Eigen::MatrixXd W;
    Eigen::VectorXd tau;
    std::size_t samples{};
};

struct IdentificationResult {
    Eigen::VectorXd theta_hat;
    Eigen::VectorXd singular_values;
    Eigen::VectorXd column_scales;
    int numerical_rank{};
    double condition_number{};
    double train_rmse_nm{};
};

IdentificationResult identifyByScaledSvd(const RegressionDataset& dataset,
                                         double relative_rank_threshold);

}  // namespace robot_id
