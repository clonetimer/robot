#include "robot_id/robot_model.hpp"
#include "robot_id/config.hpp"
#include "pinocchio_model_loader.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody.hpp>

#include <stdexcept>
#include <string>

namespace robot_id {
namespace {
void checkVectorSize(const Eigen::VectorXd& x, Eigen::Index expected, const char* name) {
    if (x.size() != expected || !x.allFinite()) {
        throw std::runtime_error(std::string("Invalid ") + name + " vector.");
    }
}
}  // namespace

struct RobotModel::Impl {
    explicit Impl(const std::string& urdf_path)
        : model(detail::loadModelFromUrdf(urdf_path)), data(model) {
        if (model.nq != model.nv) {
            throw std::runtime_error("MVP only supports fixed-base scalar joints (nq == nv).");
        }

        dynamic_parameters.resize(
            config::kParametersPerLink * static_cast<Eigen::Index>(model.njoints - 1));
        for (pinocchio::JointIndex i = 1;
             i < static_cast<pinocchio::JointIndex>(model.njoints); ++i) {
            dynamic_parameters.segment<config::kParametersPerLink>(
                config::kParametersPerLink * static_cast<Eigen::Index>(i - 1)) =
                model.inertias[i].toDynamicParameters();
        }
        if (!dynamic_parameters.allFinite()) {
            throw std::runtime_error("URDF produced non-finite dynamic parameters.");
        }
    }

    pinocchio::Model model;
    mutable pinocchio::Data data;
    Eigen::VectorXd dynamic_parameters;
};

RobotModel::RobotModel(const std::string& urdf_path)
    : impl_(std::make_unique<Impl>(urdf_path)) {}

RobotModel::~RobotModel() = default;
RobotModel::RobotModel(RobotModel&&) noexcept = default;
RobotModel& RobotModel::operator=(RobotModel&&) noexcept = default;

int RobotModel::dof() const { return impl_->model.nv; }
int RobotModel::parameterCount() const { return static_cast<int>(impl_->dynamic_parameters.size()); }
const Eigen::VectorXd& RobotModel::dynamicParameters() const { return impl_->dynamic_parameters; }
const Eigen::VectorXd& RobotModel::lowerPositionLimits() const { return impl_->model.lowerPositionLimit; }
const Eigen::VectorXd& RobotModel::upperPositionLimits() const { return impl_->model.upperPositionLimit; }
const Eigen::VectorXd& RobotModel::velocityLimits() const { return impl_->model.upperVelocityLimit; }
const Eigen::VectorXd& RobotModel::effortLimits() const { return impl_->model.upperEffortLimit; }

Eigen::MatrixXd RobotModel::torqueRegressor(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& v,
                                             const Eigen::VectorXd& a) const {
    checkVectorSize(q, impl_->model.nq, "q");
    checkVectorSize(v, impl_->model.nv, "v");
    checkVectorSize(a, impl_->model.nv, "a");
    return pinocchio::computeJointTorqueRegressor(impl_->model, impl_->data, q, v, a);
}

Eigen::VectorXd RobotModel::inverseDynamics(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& v,
                                             const Eigen::VectorXd& a) const {
    checkVectorSize(q, impl_->model.nq, "q");
    checkVectorSize(v, impl_->model.nv, "v");
    checkVectorSize(a, impl_->model.nv, "a");
    return pinocchio::rnea(impl_->model, impl_->data, q, v, a);
}

Eigen::VectorXd RobotModel::forwardDynamics(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& v,
                                             const Eigen::VectorXd& tau) const {
    checkVectorSize(q, impl_->model.nq, "q");
    checkVectorSize(v, impl_->model.nv, "v");
    checkVectorSize(tau, impl_->model.nv, "tau");
    return pinocchio::aba(impl_->model, impl_->data, q, v, tau);
}

Eigen::VectorXd RobotModel::integrate(const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& delta_q) const {
    checkVectorSize(q, impl_->model.nq, "q");
    checkVectorSize(delta_q, impl_->model.nv, "delta_q");
    return pinocchio::integrate(impl_->model, q, delta_q);
}

}  // namespace robot_id
