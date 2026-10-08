#include "pinocchio_model_loader.hpp"

#include <tinyxml2.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace robot_id::detail {
namespace {
Eigen::Vector3d parseVector3(const char* text,
                             const Eigen::Vector3d& default_value,
                             const char* field_name) {
    if (text == nullptr) {
        return default_value;
    }
    std::istringstream in(text);
    Eigen::Vector3d value;
    if (!(in >> value.x() >> value.y() >> value.z())) {
        throw std::runtime_error(std::string("Invalid URDF vector for ") + field_name + ": " + text);
    }
    std::string trailing;
    if (in >> trailing) {
        throw std::runtime_error(std::string("Too many values in URDF vector for ") + field_name + ": " + text);
    }
    return value;
}

pinocchio::SE3 parseOrigin(const tinyxml2::XMLElement* parent) {
    const tinyxml2::XMLElement* origin = parent == nullptr ? nullptr : parent->FirstChildElement("origin");
    if (origin == nullptr) {
        return pinocchio::SE3::Identity();
    }

    const Eigen::Vector3d xyz = parseVector3(origin->Attribute("xyz"), Eigen::Vector3d::Zero(), "origin xyz");
    const Eigen::Vector3d rpy = parseVector3(origin->Attribute("rpy"), Eigen::Vector3d::Zero(), "origin rpy");

    // URDF rpy is fixed-axis roll-pitch-yaw, equivalent to Rz(yaw) Ry(pitch) Rx(roll).
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    return pinocchio::SE3(rotation, xyz);
}

double requiredDoubleAttribute(const tinyxml2::XMLElement* element,
                               const char* name,
                               const char* context) {
    double value = 0.0;
    if (element == nullptr || element->QueryDoubleAttribute(name, &value) != tinyxml2::XML_SUCCESS ||
        !std::isfinite(value)) {
        throw std::runtime_error(std::string("Missing/invalid URDF attribute ") + context + "." + name);
    }
    return value;
}

pinocchio::Inertia parseLinkInertia(const tinyxml2::XMLElement* link) {
    const tinyxml2::XMLElement* inertial = link->FirstChildElement("inertial");
    if (inertial == nullptr) {
        return pinocchio::Inertia::Zero();
    }

    const tinyxml2::XMLElement* mass_element = inertial->FirstChildElement("mass");
    const double mass = requiredDoubleAttribute(mass_element, "value", "mass");
    if (!(mass > 0.0)) {
        throw std::runtime_error("AeroArm6 moving-link mass must be positive.");
    }

    const tinyxml2::XMLElement* inertia_element = inertial->FirstChildElement("inertia");
    if (inertia_element == nullptr) {
        throw std::runtime_error("Moving link is missing URDF inertia element.");
    }

    Eigen::Matrix3d inertia_in_inertial_frame;
    const double ixx = requiredDoubleAttribute(inertia_element, "ixx", "inertia");
    const double ixy = requiredDoubleAttribute(inertia_element, "ixy", "inertia");
    const double ixz = requiredDoubleAttribute(inertia_element, "ixz", "inertia");
    const double iyy = requiredDoubleAttribute(inertia_element, "iyy", "inertia");
    const double iyz = requiredDoubleAttribute(inertia_element, "iyz", "inertia");
    const double izz = requiredDoubleAttribute(inertia_element, "izz", "inertia");
    inertia_in_inertial_frame <<
        ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz;

    const pinocchio::SE3 inertial_origin = parseOrigin(inertial);
    const Eigen::Matrix3d inertia_at_com_in_link =
        inertial_origin.rotation() * inertia_in_inertial_frame * inertial_origin.rotation().transpose();

    if (!inertia_at_com_in_link.allFinite() ||
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(inertia_at_com_in_link).eigenvalues().minCoeff() <= 0.0) {
        throw std::runtime_error("Moving link has non-positive-definite COM inertia.");
    }

    return pinocchio::Inertia(mass, inertial_origin.translation(), inertia_at_com_in_link);
}

}  // namespace

pinocchio::Model loadModelFromUrdf(const std::string& urdf_path) {
    tinyxml2::XMLDocument document;
    const tinyxml2::XMLError load_result = document.LoadFile(urdf_path.c_str());
    if (load_result != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to load URDF: " + urdf_path + " (tinyxml2 error " +
                                 std::to_string(static_cast<int>(load_result)) + ")");
    }

    const tinyxml2::XMLElement* robot = document.FirstChildElement("robot");
    if (robot == nullptr) {
        throw std::runtime_error("URDF does not contain a <robot> root element.");
    }

    std::unordered_map<std::string, const tinyxml2::XMLElement*> links;
    for (const tinyxml2::XMLElement* link = robot->FirstChildElement("link");
         link != nullptr; link = link->NextSiblingElement("link")) {
        const char* name = link->Attribute("name");
        if (name == nullptr || std::string(name).empty()) {
            throw std::runtime_error("URDF link without a name.");
        }
        links.emplace(name, link);
    }
    if (links.empty()) {
        throw std::runtime_error("URDF contains no links.");
    }

    pinocchio::Model model;
    std::unordered_map<std::string, pinocchio::JointIndex> supporting_joint;
    supporting_joint.emplace("base_link", 0);

    int revolute_count = 0;
    for (const tinyxml2::XMLElement* joint = robot->FirstChildElement("joint");
         joint != nullptr; joint = joint->NextSiblingElement("joint")) {
        const char* type = joint->Attribute("type");
        const char* joint_name = joint->Attribute("name");
        if (type == nullptr || std::string(type) != "revolute") {
            throw std::runtime_error("MVP URDF loader supports revolute joints only.");
        }
        if (joint_name == nullptr || std::string(joint_name).empty()) {
            throw std::runtime_error("URDF joint without a name.");
        }

        const tinyxml2::XMLElement* parent = joint->FirstChildElement("parent");
        const tinyxml2::XMLElement* child = joint->FirstChildElement("child");
        const char* parent_link = parent == nullptr ? nullptr : parent->Attribute("link");
        const char* child_link = child == nullptr ? nullptr : child->Attribute("link");
        if (parent_link == nullptr || child_link == nullptr) {
            throw std::runtime_error("URDF joint is missing parent/child link.");
        }

        const auto parent_it = supporting_joint.find(parent_link);
        if (parent_it == supporting_joint.end()) {
            throw std::runtime_error("URDF joints must be ordered parent-before-child for the MVP loader.");
        }
        const auto child_link_it = links.find(child_link);
        if (child_link_it == links.end()) {
            throw std::runtime_error("URDF joint references an unknown child link.");
        }

        const tinyxml2::XMLElement* axis_element = joint->FirstChildElement("axis");
        Eigen::Vector3d axis = parseVector3(
            axis_element == nullptr ? nullptr : axis_element->Attribute("xyz"),
            Eigen::Vector3d::UnitX(), "joint axis");
        const double axis_norm = axis.norm();
        if (!(axis_norm > 1e-12) || !std::isfinite(axis_norm)) {
            throw std::runtime_error("URDF joint axis must be finite and non-zero.");
        }
        axis /= axis_norm;

        const tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
        if (limit == nullptr) {
            throw std::runtime_error("Revolute joint is missing URDF limits.");
        }
        const double lower = requiredDoubleAttribute(limit, "lower", "limit");
        const double upper = requiredDoubleAttribute(limit, "upper", "limit");
        const double effort = requiredDoubleAttribute(limit, "effort", "limit");
        const double velocity = requiredDoubleAttribute(limit, "velocity", "limit");
        if (!(lower < upper) || !(effort > 0.0) || !(velocity > 0.0)) {
            throw std::runtime_error("Invalid revolute-joint limits in URDF.");
        }

        const pinocchio::JointIndex joint_id = model.addJoint(
            parent_it->second,
            pinocchio::JointModelRevoluteUnaligned(axis),
            parseOrigin(joint),
            joint_name);

        const Eigen::Index v_index = static_cast<Eigen::Index>(model.joints[joint_id].idx_v());
        const Eigen::Index q_index = static_cast<Eigen::Index>(model.joints[joint_id].idx_q());
        model.lowerEffortLimit(v_index) = -effort;
        model.upperEffortLimit(v_index) = effort;
        model.lowerVelocityLimit(v_index) = -velocity;
        model.upperVelocityLimit(v_index) = velocity;
        model.lowerPositionLimit(q_index) = lower;
        model.upperPositionLimit(q_index) = upper;

        model.appendBodyToJoint(joint_id, parseLinkInertia(child_link_it->second));
        supporting_joint.emplace(child_link, joint_id);
        ++revolute_count;
    }

    if (revolute_count == 0) {
        throw std::runtime_error("URDF contains no revolute joints.");
    }
    return model;
}


}  // namespace robot_id::detail
