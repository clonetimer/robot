#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>

namespace robot_id {

class RobotModel {
public:
    explicit RobotModel(const std::string& urdf_path);
    ~RobotModel();

    RobotModel(const RobotModel&) = delete;
    RobotModel& operator=(const RobotModel&) = delete;
    RobotModel(RobotModel&&) noexcept;
    RobotModel& operator=(RobotModel&&) noexcept;

    int dof() const;
    int parameterCount() const;

    const Eigen::VectorXd& dynamicParameters() const;
    const Eigen::VectorXd& lowerPositionLimits() const;
    const Eigen::VectorXd& upperPositionLimits() const;
    const Eigen::VectorXd& velocityLimits() const;
    const Eigen::VectorXd& effortLimits() const;

    Eigen::MatrixXd torqueRegressor(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& v,
                                    const Eigen::VectorXd& a) const;

    Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& v,
                                    const Eigen::VectorXd& a) const;

    Eigen::VectorXd forwardDynamics(const Eigen::VectorXd& q,
                                    const Eigen::VectorXd& v,
                                    const Eigen::VectorXd& tau) const;

    Eigen::VectorXd integrate(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& delta_q) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robot_id
