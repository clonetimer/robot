#pragma once

#include <pinocchio/multibody.hpp>

#include <string>

namespace robot_id::detail {
pinocchio::Model loadModelFromUrdf(const std::string& urdf_path);
}
