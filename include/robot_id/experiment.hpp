#pragma once

#include "robot_id/robot_model.hpp"
#include "robot_id/trajectory.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace robot_id {

enum class ControllerKind {
    PdOnly,
    ComputedTorqueNominal,
    ComputedTorqueIdentified,
};

struct ControlMetrics {
    ControllerKind kind{};
    Eigen::VectorXd joint_rmse_rad;
    double overall_rmse_rad{};
    double max_abs_error_rad{};

    // Diagnostics only; they do not add acceptance criteria.
    double saturation_fraction{};
    double peak_unsaturated_effort_ratio{};
    double min_position_limit_margin_rad{};
    double peak_velocity_limit_ratio{};
};

struct ControlTraceSample {
    double t{};
    Eigen::VectorXd q;
    Eigen::VectorXd q_des;
    Eigen::VectorXd tau;
};

struct ControlRun {
    ControlMetrics metrics;
    std::vector<ControlTraceSample> trace;
};

struct AcceptanceSummary {
    double a0_regressor_relative_error{};
    double a1_unseen_torque_nrmse{};
    double a2_ct_nominal_rmse_deg{};
    double a2_ct_identified_rmse_deg{};
    bool a0_pass{};
    bool a1_pass{};
    bool a2_pass{};

    bool allPass() const { return a0_pass && a1_pass && a2_pass; }
};

ControlRun simulateTracking(const RobotModel& plant,
                            const MultiSineTrajectory& desired,
                            ControllerKind kind,
                            const Eigen::VectorXd& theta_nominal,
                            const Eigen::VectorXd& theta_identified,
                            double duration_s,
                            double step_s,
                            int log_stride = 10);

std::string controllerName(ControllerKind kind);

void writeVectorCsv(const std::string& path,
                    const Eigen::VectorXd& truth,
                    const Eigen::VectorXd& nominal,
                    const Eigen::VectorXd& identified);

void writeValidationCsv(const std::string& path,
                        const Eigen::VectorXd& rmse_identified,
                        const Eigen::VectorXd& nrmse_identified,
                        const Eigen::VectorXd& rmse_nominal,
                        const Eigen::VectorXd& nrmse_nominal,
                        double overall_rmse_identified_nm,
                        double overall_nrmse_identified,
                        double overall_rmse_nominal_nm,
                        double overall_nrmse_nominal);

void writeControlMetricsCsv(const std::string& path,
                            const std::vector<ControlRun>& runs);

void writeControlTraceCsv(const std::string& path,
                          const ControlRun& run);

void writeAcceptanceCsv(const std::string& path,
                        const AcceptanceSummary& summary);

void writeSingularValuesCsv(const std::string& path,
                            const Eigen::VectorXd& singular_values,
                            double relative_rank_threshold);

}  // namespace robot_id
