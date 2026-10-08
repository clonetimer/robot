#include "robot_id/experiment.hpp"
#include "robot_id/config.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace robot_id {
namespace {
constexpr double kRadToDeg = 57.2957795130823208768;

Eigen::VectorXd saturateTorque(const Eigen::VectorXd& tau, const Eigen::VectorXd& limit) {
    Eigen::VectorXd out = tau;
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        const double lim = (i < limit.size() && std::isfinite(limit(i)) && limit(i) > 0.0)
            ? limit(i)
            : 1e9;
        out(i) = std::clamp(out(i), -lim, lim);
    }
    return out;
}

double finitePositiveLimit(const Eigen::VectorXd& limit, Eigen::Index i) {
    return (i < limit.size() && std::isfinite(limit(i)) && limit(i) > 0.0)
        ? limit(i)
        : 1e9;
}

void ensureParentDirectory(const std::string& path) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }
}
}  // namespace

std::string controllerName(ControllerKind kind) {
    switch (kind) {
        case ControllerKind::PdOnly: return "pd_only";
        case ControllerKind::ComputedTorqueNominal: return "ct_nominal";
        case ControllerKind::ComputedTorqueIdentified: return "ct_identified";
    }
    return "unknown";
}

ControlRun simulateTracking(const RobotModel& plant,
                            const MultiSineTrajectory& desired,
                            ControllerKind kind,
                            const Eigen::VectorXd& theta_nominal,
                            const Eigen::VectorXd& theta_identified,
                            double duration_s,
                            double step_s,
                            int log_stride) {
    const int n = plant.dof();
    if (n != desired.dof() || theta_nominal.size() != plant.parameterCount() ||
        theta_identified.size() != plant.parameterCount() || !theta_nominal.allFinite() ||
        !theta_identified.allFinite() || !std::isfinite(duration_s) || duration_s <= 0.0 ||
        !std::isfinite(step_s) || step_s <= 0.0 || log_stride <= 0) {
        throw std::runtime_error("Invalid control simulation arguments.");
    }

    const auto x0 = desired.sample(0.0);
    Eigen::VectorXd q = x0.q;
    Eigen::VectorXd v = x0.v;
    Eigen::VectorXd initial_offset(n);
    for (int j = 0; j < n; ++j) {
        initial_offset(j) = config::kInitialPositionOffsetRad.at(static_cast<std::size_t>(j));
    }
    q += initial_offset;
    v.setZero();

    const Eigen::VectorXd kp = Eigen::VectorXd::Constant(n, config::kCtcKp);
    const Eigen::VectorXd kd = Eigen::VectorXd::Constant(n, config::kCtcKd);
    const Eigen::VectorXd kp_pd = Eigen::VectorXd::Constant(n, config::kPdKp);
    const Eigen::VectorXd kd_pd = Eigen::VectorXd::Constant(n, config::kPdKd);

    const std::size_t steps = static_cast<std::size_t>(std::floor(duration_s / step_s)) + 1;
    Eigen::VectorXd err_sq = Eigen::VectorXd::Zero(n);
    double max_abs_error = 0.0;
    std::size_t saturated_joint_commands = 0;
    double peak_unsaturated_effort_ratio = 0.0;
    double min_position_limit_margin = std::numeric_limits<double>::infinity();
    double peak_velocity_limit_ratio = 0.0;

    ControlRun run;
    run.metrics.kind = kind;
    run.trace.reserve(steps / static_cast<std::size_t>(log_stride) + 1);

    const Eigen::VectorXd& effort_limits = plant.effortLimits();
    const Eigen::VectorXd& velocity_limits = plant.velocityLimits();
    const Eigen::VectorXd& lower_limits = plant.lowerPositionLimits();
    const Eigen::VectorXd& upper_limits = plant.upperPositionLimits();

    for (std::size_t i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * step_s;
        const auto xd = desired.sample(t);
        const Eigen::VectorXd e = xd.q - q;
        const Eigen::VectorXd edot = xd.v - v;
        err_sq += e.array().square().matrix();
        max_abs_error = std::max(max_abs_error, e.cwiseAbs().maxCoeff());

        for (int j = 0; j < n; ++j) {
            if (j < lower_limits.size() && j < upper_limits.size() &&
                std::isfinite(lower_limits(j)) && std::isfinite(upper_limits(j))) {
                const double margin = std::min(q(j) - lower_limits(j), upper_limits(j) - q(j));
                min_position_limit_margin = std::min(min_position_limit_margin, margin);
            }
            const double vlim = finitePositiveLimit(velocity_limits, j);
            peak_velocity_limit_ratio = std::max(peak_velocity_limit_ratio, std::abs(v(j)) / vlim);
        }

        Eigen::VectorXd tau(n);
        if (kind == ControllerKind::PdOnly) {
            tau = kp_pd.cwiseProduct(e) + kd_pd.cwiseProduct(edot);
        } else {
            const Eigen::VectorXd a_ref = xd.a + kd.cwiseProduct(edot) + kp.cwiseProduct(e);
            const Eigen::MatrixXd Y = plant.torqueRegressor(q, v, a_ref);
            const Eigen::VectorXd& theta = (kind == ControllerKind::ComputedTorqueIdentified)
                ? theta_identified
                : theta_nominal;
            tau = Y * theta;
        }
        if (!tau.allFinite()) {
            throw std::runtime_error("Controller produced NaN/Inf torque.");
        }
        for (int j = 0; j < n; ++j) {
            const double lim = finitePositiveLimit(effort_limits, j);
            const double ratio = std::abs(tau(j)) / lim;
            peak_unsaturated_effort_ratio = std::max(peak_unsaturated_effort_ratio, ratio);
            if (ratio > 1.0) {
                ++saturated_joint_commands;
            }
        }
        tau = saturateTorque(tau, effort_limits);

        if (i % static_cast<std::size_t>(log_stride) == 0) {
            run.trace.push_back({t, q, xd.q, tau});
        }

        const Eigen::VectorXd a = plant.forwardDynamics(q, v, tau);
        if (!a.allFinite()) {
            throw std::runtime_error("Plant forward dynamics produced NaN/Inf acceleration.");
        }
        // Semi-implicit Euler: velocity is updated before configuration.
        v += a * step_s;
        q = plant.integrate(q, v * step_s);
        if (!q.allFinite() || !v.allFinite()) {
            throw std::runtime_error("Closed-loop state became NaN/Inf.");
        }
    }

    run.metrics.joint_rmse_rad = (err_sq / static_cast<double>(steps)).array().sqrt().matrix();
    run.metrics.overall_rmse_rad = std::sqrt(err_sq.sum() / static_cast<double>(steps * n));
    run.metrics.max_abs_error_rad = max_abs_error;
    run.metrics.saturation_fraction = static_cast<double>(saturated_joint_commands) /
        static_cast<double>(steps * static_cast<std::size_t>(n));
    run.metrics.peak_unsaturated_effort_ratio = peak_unsaturated_effort_ratio;
    run.metrics.min_position_limit_margin_rad = std::isfinite(min_position_limit_margin)
        ? min_position_limit_margin
        : std::numeric_limits<double>::quiet_NaN();
    run.metrics.peak_velocity_limit_ratio = peak_velocity_limit_ratio;
    return run;
}

void writeVectorCsv(const std::string& path,
                    const Eigen::VectorXd& truth,
                    const Eigen::VectorXd& nominal,
                    const Eigen::VectorXd& identified) {
    if (truth.size() != nominal.size() || truth.size() != identified.size() ||
        truth.size() % config::kParametersPerLink != 0 || !truth.allFinite() ||
        !nominal.allFinite() || !identified.allFinite()) {
        throw std::runtime_error("Parameter CSV vectors have inconsistent dimensions.");
    }
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open parameter CSV: " + path);
    }
    static constexpr const char* kNames[config::kParametersPerLink] = {
        "m", "mcx", "mcy", "mcz", "Ixx", "Ixy", "Iyy", "Ixz", "Iyz", "Izz"};
    os << "index,link,parameter,theta_true,theta_nominal,theta_identified\n";
    os << std::setprecision(12);
    for (Eigen::Index i = 0; i < truth.size(); ++i) {
        const Eigen::Index link = i / config::kParametersPerLink;
        const Eigen::Index local = i % config::kParametersPerLink;
        os << i << ',' << (link + 1) << ',' << kNames[local] << ','
           << truth(i) << ',' << nominal(i) << ',' << identified(i) << '\n';
    }
}

void writeValidationCsv(const std::string& path,
                        const Eigen::VectorXd& rmse_identified,
                        const Eigen::VectorXd& nrmse_identified,
                        const Eigen::VectorXd& rmse_nominal,
                        const Eigen::VectorXd& nrmse_nominal,
                        double overall_rmse_identified_nm,
                        double overall_nrmse_identified,
                        double overall_rmse_nominal_nm,
                        double overall_nrmse_nominal) {
    if (rmse_identified.size() != nrmse_identified.size() ||
        rmse_identified.size() != rmse_nominal.size() ||
        rmse_identified.size() != nrmse_nominal.size() ||
        !rmse_identified.allFinite() || !nrmse_identified.allFinite() ||
        !rmse_nominal.allFinite() || !nrmse_nominal.allFinite() ||
        !std::isfinite(overall_rmse_identified_nm) || !std::isfinite(overall_nrmse_identified) ||
        !std::isfinite(overall_rmse_nominal_nm) || !std::isfinite(overall_nrmse_nominal) ||
        overall_rmse_identified_nm < 0.0 || overall_nrmse_identified < 0.0 ||
        overall_rmse_nominal_nm < 0.0 || overall_nrmse_nominal < 0.0) {
        throw std::runtime_error("Validation CSV metrics have inconsistent dimensions or invalid values.");
    }
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open validation CSV: " + path);
    }
    os << "joint,identified_rmse_nm,identified_nrmse,nominal_rmse_nm,nominal_nrmse\n";
    os << std::setprecision(12);
    for (Eigen::Index i = 0; i < rmse_identified.size(); ++i) {
        os << (i + 1) << ',' << rmse_identified(i) << ',' << nrmse_identified(i) << ','
           << rmse_nominal(i) << ',' << nrmse_nominal(i) << '\n';
    }
    // FR-05 requires both per-joint and overall validation metrics in the
    // machine-readable result artifact. Keep the same schema and add an
    // explicit aggregate row instead of relying on console output.
    os << "overall," << overall_rmse_identified_nm << ',' << overall_nrmse_identified << ','
       << overall_rmse_nominal_nm << ',' << overall_nrmse_nominal << '\n';
}

void writeControlMetricsCsv(const std::string& path,
                            const std::vector<ControlRun>& runs) {
    Eigen::Index expected_joints = -1;
    for (const auto& run : runs) {
        const auto& m = run.metrics;
        if (expected_joints < 0) expected_joints = m.joint_rmse_rad.size();
        if (m.joint_rmse_rad.size() != expected_joints || !m.joint_rmse_rad.allFinite() ||
            !std::isfinite(m.overall_rmse_rad) || !std::isfinite(m.max_abs_error_rad) ||
            !std::isfinite(m.saturation_fraction) ||
            !std::isfinite(m.peak_unsaturated_effort_ratio) ||
            !std::isfinite(m.peak_velocity_limit_ratio) ||
            m.saturation_fraction < 0.0 || m.saturation_fraction > 1.0) {
            throw std::runtime_error("Control metrics contain invalid values or dimensions.");
        }
    }
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open control-metrics CSV: " + path);
    }
    os << "controller,overall_rmse_rad,overall_rmse_deg,max_abs_error_rad,max_abs_error_deg"
          ",saturation_fraction,peak_unsaturated_effort_ratio,min_position_limit_margin_rad"
          ",peak_velocity_limit_ratio";
    if (!runs.empty()) {
        for (Eigen::Index j = 0; j < runs.front().metrics.joint_rmse_rad.size(); ++j) {
            os << ",joint" << (j + 1) << "_rmse_rad";
        }
    }
    os << '\n';
    os << std::setprecision(12);
    for (const auto& run : runs) {
        os << controllerName(run.metrics.kind) << ','
           << run.metrics.overall_rmse_rad << ',' << run.metrics.overall_rmse_rad * kRadToDeg << ','
           << run.metrics.max_abs_error_rad << ',' << run.metrics.max_abs_error_rad * kRadToDeg << ','
           << run.metrics.saturation_fraction << ','
           << run.metrics.peak_unsaturated_effort_ratio << ','
           << run.metrics.min_position_limit_margin_rad << ','
           << run.metrics.peak_velocity_limit_ratio;
        for (Eigen::Index j = 0; j < run.metrics.joint_rmse_rad.size(); ++j) {
            os << ',' << run.metrics.joint_rmse_rad(j);
        }
        os << '\n';
    }
}

void writeControlTraceCsv(const std::string& path,
                          const ControlRun& run) {
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open control-trace CSV: " + path);
    }
    const int n = run.trace.empty() ? 0 : static_cast<int>(run.trace.front().q.size());
    for (const auto& sample : run.trace) {
        if (!std::isfinite(sample.t) || sample.q.size() != n || sample.q_des.size() != n ||
            sample.tau.size() != n || !sample.q.allFinite() || !sample.q_des.allFinite() ||
            !sample.tau.allFinite()) {
            throw std::runtime_error("Control trace contains invalid values or dimensions.");
        }
    }
    os << "t";
    for (int j = 0; j < n; ++j) os << ",q" << (j + 1);
    for (int j = 0; j < n; ++j) os << ",q_des" << (j + 1);
    for (int j = 0; j < n; ++j) os << ",tau" << (j + 1);
    os << '\n';
    os << std::setprecision(12);
    for (const auto& s : run.trace) {
        os << s.t;
        for (int j = 0; j < n; ++j) os << ',' << s.q(j);
        for (int j = 0; j < n; ++j) os << ',' << s.q_des(j);
        for (int j = 0; j < n; ++j) os << ',' << s.tau(j);
        os << '\n';
    }
}

void writeAcceptanceCsv(const std::string& path,
                        const AcceptanceSummary& summary) {
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open acceptance CSV: " + path);
    }
    os << "criterion,metric,threshold_or_comparison,pass\n";
    os << std::setprecision(12);
    os << "A0," << summary.a0_regressor_relative_error << ",<"
       << config::kA0MaxRegressorRelativeError << ',' << (summary.a0_pass ? "PASS" : "FAIL") << '\n';
    os << "A1," << summary.a1_unseen_torque_nrmse << ",<"
       << config::kA1MaxTorqueNrmse << ',' << (summary.a1_pass ? "PASS" : "FAIL") << '\n';
    os << "A2," << summary.a2_ct_identified_rmse_deg
       << ",<" << config::kA2MaxTrackingRmseDeg << " deg and < nominal("
       << summary.a2_ct_nominal_rmse_deg << " deg),"
       << (summary.a2_pass ? "PASS" : "FAIL") << '\n';
}

void writeSingularValuesCsv(const std::string& path,
                            const Eigen::VectorXd& singular_values,
                            double relative_rank_threshold) {
    if (!singular_values.allFinite() || !std::isfinite(relative_rank_threshold) ||
        relative_rank_threshold <= 0.0) {
        throw std::runtime_error("Invalid singular-value CSV data.");
    }
    ensureParentDirectory(path);
    std::ofstream os(path);
    if (!os) {
        throw std::runtime_error("Failed to open singular-value CSV: " + path);
    }
    os << "index,singular_value,relative_to_max,retained\n";
    os << std::setprecision(12);
    const double smax = singular_values.size() > 0 ? singular_values(0) : 0.0;
    for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
        const double relative = smax > 0.0 ? singular_values(i) / smax : 0.0;
        os << i << ',' << singular_values(i) << ',' << relative << ','
           << (relative > relative_rank_threshold ? 1 : 0) << '\n';
    }
}

}  // namespace robot_id
