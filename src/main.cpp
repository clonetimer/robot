#include "robot_id/config.hpp"
#include "robot_id/dynamic_parameters.hpp"
#include "robot_id/experiment.hpp"
#include "robot_id/identification.hpp"
#include "robot_id/robot_model.hpp"
#include "robot_id/random.hpp"
#include "robot_id/trajectory.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

void printUsage(const char* exe) {
    std::cout << "Usage: " << exe << " [--urdf path] [--out directory]\n";
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string urdf = "models/aeroarm6.urdf";
        std::string out_dir = "results";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--urdf" && i + 1 < argc) {
                urdf = argv[++i];
            } else if (arg == "--out" && i + 1 < argc) {
                out_dir = argv[++i];
            } else if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            } else {
                throw std::runtime_error("Unknown or incomplete argument: " + arg);
            }
        }

        std::filesystem::create_directories(out_dir);
        robot_id::RobotModel robot(urdf);
        if (robot.dof() != robot_id::config::kDof ||
            robot.parameterCount() != robot_id::config::kFullParameterCount) {
            throw std::runtime_error("MVP is frozen to a fixed-base 6R / 60-inertial-parameter model.");
        }

        std::cout << "[1/5] Model loaded: dof=" << robot.dof()
                  << ", full inertial parameters=" << robot.parameterCount() << "\n";

        // Runtime A0: evaluate the identity over the same frozen deterministic
        // 100-state protocol used by the dedicated integration test.
        robot_id::DeterministicNormalGenerator a0_rng(robot_id::config::kA0RandomSeed);
        double identity_rel_err = 0.0;
        for (int sample = 0; sample < robot_id::config::kA0StateSamples; ++sample) {
            Eigen::VectorXd q(robot.dof()), v(robot.dof()), a(robot.dof());
            for (int j = 0; j < robot.dof(); ++j) {
                q(j) = (2.0 * a0_rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsPositionRad;
                v(j) = (2.0 * a0_rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsVelocityRadS;
                a(j) = (2.0 * a0_rng.uniformOpen01() - 1.0) * robot_id::config::kA0MaxAbsAccelerationRadS2;
            }
            const Eigen::VectorXd tau_rnea = robot.inverseDynamics(q, v, a);
            const Eigen::VectorXd tau_reg = robot.torqueRegressor(q, v, a)
                                          * robot.dynamicParameters();
            const double rel = (tau_rnea - tau_reg).norm() / std::max(tau_rnea.norm(), 1e-12);
            identity_rel_err = std::max(identity_rel_err, rel);
        }
        std::cout << "      worst regressor identity relative error over "
                  << robot_id::config::kA0StateSamples << " states = " << std::scientific
                  << identity_rel_err << std::defaultfloat << "\n";
        if (identity_rel_err > robot_id::config::kA0MaxRegressorRelativeError) {
            throw std::runtime_error("Regressor identity check failed.");
        }

        std::cout << "[2/5] Generating deterministic multi-sine identification dataset...\n";
        const auto id_traj = robot_id::makeIdentificationTrajectory();
        const double id_duration = static_cast<double>(robot_id::config::kIdentificationPeriods)
                                 * 2.0 * kPi / id_traj.fundamental();
        const auto dataset = robot_id::generateSyntheticDataset(
            robot, id_traj, id_duration,
            robot_id::config::kIdentificationSamplePeriodS,
            robot_id::config::kTorqueNoiseStdNm,
            robot_id::config::kRandomSeed);
        std::cout << "      samples=" << dataset.samples
                  << ", stacked W=" << dataset.W.rows() << "x" << dataset.W.cols() << "\n";

        std::cout << "[3/5] Solving scaled truncated-SVD least squares...\n";
        const auto id = robot_id::identifyByScaledSvd(
            dataset, robot_id::config::kSvdRelativeRankThreshold);
        std::cout << "      numerical rank=" << id.numerical_rank << "/" << robot.parameterCount()
                  << ", scaled condition number=" << id.condition_number
                  << ", train RMSE=" << id.train_rmse_nm << " Nm\n";

        const Eigen::VectorXd theta_true = robot.dynamicParameters();
        const Eigen::VectorXd theta_nominal = robot_id::makeNominalMismatchedParameters(theta_true);
        robot_id::writeVectorCsv(out_dir + "/parameters.csv", theta_true, theta_nominal, id.theta_hat);
        robot_id::writeSingularValuesCsv(out_dir + "/singular_values.csv",
                                         id.singular_values,
                                         robot_id::config::kSvdRelativeRankThreshold);

        std::cout << "[4/5] Validating torque prediction on an unseen trajectory...\n";
        const auto val_traj = robot_id::makeValidationTrajectory();
        const auto val_id = robot_id::validateTorquePrediction(
            robot, val_traj, id.theta_hat,
            robot_id::config::kValidationDurationS,
            robot_id::config::kValidationSamplePeriodS);
        const auto val_nom = robot_id::validateTorquePrediction(
            robot, val_traj, theta_nominal,
            robot_id::config::kValidationDurationS,
            robot_id::config::kValidationSamplePeriodS);
        robot_id::writeValidationCsv(out_dir + "/validation_metrics.csv",
                                     val_id.rmse_nm, val_id.nrmse,
                                     val_nom.rmse_nm, val_nom.nrmse,
                                     val_id.overall_rmse_nm, val_id.overall_nrmse,
                                     val_nom.overall_rmse_nm, val_nom.overall_nrmse);
        std::cout << "      identified overall RMSE=" << val_id.overall_rmse_nm
                  << " Nm, NRMSE=" << 100.0 * val_id.overall_nrmse << "%\n";
        std::cout << "      nominal    overall RMSE=" << val_nom.overall_rmse_nm
                  << " Nm, NRMSE=" << 100.0 * val_nom.overall_nrmse << "%\n";

        std::cout << "[5/5] Closed-loop tracking with true plant (ABA), 1 ms simulation step...\n";
        const auto control_traj = robot_id::makeControlTrajectory();
        std::vector<robot_id::ControlRun> runs;
        for (const auto kind : {robot_id::ControllerKind::PdOnly,
                                robot_id::ControllerKind::ComputedTorqueNominal,
                                robot_id::ControllerKind::ComputedTorqueIdentified}) {
            auto run = robot_id::simulateTracking(robot, control_traj, kind,
                                                  theta_nominal, id.theta_hat,
                                                  robot_id::config::kControlDurationS,
                                                  robot_id::config::kControlStepS,
                                                  robot_id::config::kControlLogStride);
            std::cout << "      " << robot_id::controllerName(kind)
                      << ": overall RMSE=" << run.metrics.overall_rmse_rad * kRadToDeg
                      << " deg, max=" << run.metrics.max_abs_error_rad * kRadToDeg
                      << " deg, saturation=" << 100.0 * run.metrics.saturation_fraction
                      << "%, peak effort ratio=" << run.metrics.peak_unsaturated_effort_ratio
                      << ", min joint-limit margin=" << run.metrics.min_position_limit_margin_rad
                      << " rad, peak velocity ratio=" << run.metrics.peak_velocity_limit_ratio
                      << "\n";
            if (run.metrics.saturation_fraction > 0.0) {
                std::cout << "        WARN: torque saturation occurred; inspect control_metrics.csv.\n";
            }
            if (std::isfinite(run.metrics.min_position_limit_margin_rad) &&
                run.metrics.min_position_limit_margin_rad < 0.0) {
                std::cout << "        WARN: a joint position limit was crossed.\n";
            }
            if (run.metrics.peak_velocity_limit_ratio > 1.0) {
                std::cout << "        WARN: a joint velocity limit was exceeded.\n";
            }
            robot_id::writeControlTraceCsv(out_dir + "/trace_" + robot_id::controllerName(kind) + ".csv",
                                           run);
            runs.push_back(std::move(run));
        }
        robot_id::writeControlMetricsCsv(out_dir + "/control_metrics.csv", runs);

        // Frozen MVP acceptance checks. These are intentionally performance-based rather than
        // element-wise parameter checks because the full 60-parameter vector is structurally redundant.
        const double ct_nominal = runs[1].metrics.overall_rmse_rad;
        const double ct_identified = runs[2].metrics.overall_rmse_rad;

        robot_id::AcceptanceSummary acceptance;
        acceptance.a0_regressor_relative_error = identity_rel_err;
        acceptance.a1_unseen_torque_nrmse = val_id.overall_nrmse;
        acceptance.a2_ct_nominal_rmse_deg = ct_nominal * kRadToDeg;
        acceptance.a2_ct_identified_rmse_deg = ct_identified * kRadToDeg;
        acceptance.a0_pass = identity_rel_err < robot_id::config::kA0MaxRegressorRelativeError;
        acceptance.a1_pass = val_id.overall_nrmse < robot_id::config::kA1MaxTorqueNrmse;
        acceptance.a2_pass = ct_identified < ct_nominal &&
            acceptance.a2_ct_identified_rmse_deg < robot_id::config::kA2MaxTrackingRmseDeg;
        robot_id::writeAcceptanceCsv(out_dir + "/acceptance.csv", acceptance);

        std::cout << "\nMVP acceptance:\n"
                  << "  A0 RNEA/regressor identity: "
                  << (acceptance.a0_pass ? "PASS" : "FAIL") << "\n"
                  << "  A1 unseen-trajectory torque NRMSE < 5%: "
                  << (acceptance.a1_pass ? "PASS" : "FAIL") << "\n"
                  << "  A2 identified CTC beats mismatched CTC and RMSE < 1 deg: "
                  << (acceptance.a2_pass ? "PASS" : "FAIL") << "\n";
        return acceptance.allPass() ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
