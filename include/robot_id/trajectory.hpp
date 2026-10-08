#pragma once

#include <Eigen/Core>

namespace robot_id {

struct TrajectoryState {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
};

class MultiSineTrajectory {
public:
    MultiSineTrajectory(Eigen::VectorXd offset,
                        Eigen::MatrixXd amplitudes,
                        Eigen::MatrixXd phases,
                        double fundamental_rad_s);

    TrajectoryState sample(double t) const;
    int dof() const { return static_cast<int>(offset_.size()); }
    int harmonics() const { return static_cast<int>(amplitudes_.cols()); }
    double fundamental() const { return fundamental_rad_s_; }

private:
    Eigen::VectorXd offset_;
    Eigen::MatrixXd amplitudes_;
    Eigen::MatrixXd phases_;
    double fundamental_rad_s_;
};

MultiSineTrajectory makeIdentificationTrajectory();
MultiSineTrajectory makeValidationTrajectory();
MultiSineTrajectory makeControlTrajectory();

}  // namespace robot_id
