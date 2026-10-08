#include "robot_id/least_squares.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace robot_id {

IdentificationResult identifyByScaledSvd(const RegressionDataset& dataset,
                                         double relative_rank_threshold) {
    if (dataset.W.rows() != dataset.tau.size() || dataset.W.rows() == 0 || dataset.W.cols() == 0 ||
        !std::isfinite(relative_rank_threshold) || relative_rank_threshold <= 0.0) {
        throw std::runtime_error("Invalid regression dataset or SVD threshold.");
    }
    if (!dataset.W.allFinite() || !dataset.tau.allFinite()) {
        throw std::runtime_error("Regression dataset contains NaN or Inf.");
    }

    IdentificationResult result;
    result.column_scales.resize(dataset.W.cols());
    Eigen::MatrixXd W_scaled = dataset.W;

    for (Eigen::Index j = 0; j < dataset.W.cols(); ++j) {
        double scale = dataset.W.col(j).norm();
        if (scale < 1e-12) {
            scale = 1.0;
        }
        result.column_scales(j) = scale;
        W_scaled.col(j) /= scale;
    }

    Eigen::BDCSVD<Eigen::MatrixXd> svd(W_scaled, Eigen::ComputeThinU | Eigen::ComputeThinV);
    result.singular_values = svd.singularValues();

    const double s_max = result.singular_values.size() > 0 ? result.singular_values(0) : 0.0;
    const double cutoff = relative_rank_threshold * s_max;
    result.numerical_rank = 0;
    for (Eigen::Index i = 0; i < result.singular_values.size(); ++i) {
        if (result.singular_values(i) > cutoff) {
            ++result.numerical_rank;
        }
    }

    Eigen::VectorXd beta = Eigen::VectorXd::Zero(dataset.W.cols());
    if (result.numerical_rank > 0) {
        const Eigen::Index r = static_cast<Eigen::Index>(result.numerical_rank);
        const Eigen::VectorXd projected = svd.matrixU().leftCols(r).transpose() * dataset.tau;
        const Eigen::VectorXd weighted = projected.cwiseQuotient(result.singular_values.head(r));
        beta = svd.matrixV().leftCols(r) * weighted;
        result.condition_number = s_max / result.singular_values(r - 1);
    } else {
        result.condition_number = std::numeric_limits<double>::infinity();
    }

    result.theta_hat = beta.cwiseQuotient(result.column_scales);
    const Eigen::VectorXd residual = dataset.W * result.theta_hat - dataset.tau;
    result.train_rmse_nm = std::sqrt(residual.squaredNorm() / static_cast<double>(residual.size()));
    return result;
}

}  // namespace robot_id
