#include "kinematics/manipulator2d.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kinematics {

Manipulator2D::Manipulator2D(double l1, double l2) : l1_(l1), l2_(l2) {
    if (l1_ <= 0.0 || l2_ <= 0.0) {
        throw std::invalid_argument("Link lengths l1 and l2 must be strictly positive.");
    }
}

EndEffectorPose Manipulator2D::computeFK(const JointState& state) const noexcept {
    return computeFK(state.theta1, state.theta2);
}

EndEffectorPose Manipulator2D::computeFK(double theta1, double theta2) const noexcept {
    double theta12 = theta1 + theta2;
    double x = l1_ * std::cos(theta1) + l2_ * std::cos(theta12);
    double y = l1_ * std::sin(theta1) + l2_ * std::sin(theta12);
    return EndEffectorPose{x, y};
}

IKResult Manipulator2D::computeIK(double x, double y, ElbowConfig config) const noexcept {
    return computeIK(EndEffectorPose{x, y}, config);
}

IKResult Manipulator2D::computeIK(const EndEffectorPose& pose, ElbowConfig config) const noexcept {
    IKResult result;

    // Check for non-finite input coordinates (NaN or Inf)
    if (std::isnan(pose.x) || std::isnan(pose.y) || std::isinf(pose.x) || std::isinf(pose.y)) {
        result.status = IKStatus::OUT_OF_REACH;
        result.joint_state = {0.0, 0.0};
        return result;
    }

    double r_sq = pose.x * pose.x + pose.y * pose.y;
    double r = std::sqrt(r_sq);

    constexpr double EPSILON = 1e-9;

    // Check for origin singularity (e.g. x=0, y=0 when l1 == l2)
    if (r < EPSILON) {
        if (std::abs(l1_ - l2_) < EPSILON) {
            result.status = IKStatus::SINGULARITY;
            result.joint_state = {0.0, 0.0};
            return result;
        } else {
            result.status = IKStatus::OUT_OF_REACH;
            result.joint_state = {0.0, 0.0};
            return result;
        }
    }

    double cos_theta2 = (r_sq - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);

    // Out-of-bounds reachability check
    if (cos_theta2 > 1.0 + EPSILON || cos_theta2 < -1.0 - EPSILON) {
        result.status = IKStatus::OUT_OF_REACH;
        result.joint_state = {0.0, 0.0};
        return result;
    }

    // Clamp cos_theta2 to valid [-1.0, 1.0] domain
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);

    double sin_theta2_mag = std::sqrt(std::max(0.0, 1.0 - cos_theta2 * cos_theta2));

    double theta2 = 0.0;
    if (config == ElbowConfig::ELBOW_UP) {
        // Positive angle solution
        theta2 = std::atan2(sin_theta2_mag, cos_theta2);
    } else {
        // Negative angle solution
        theta2 = std::atan2(-sin_theta2_mag, cos_theta2);
    }

    double k1 = l1_ + l2_ * std::cos(theta2);
    double k2 = l2_ * std::sin(theta2);

    double theta1 = std::atan2(pose.y, pose.x) - std::atan2(k2, k1);

    // Normalize theta1 to [-pi, pi]
    theta1 = std::atan2(std::sin(theta1), std::cos(theta1));

    result.status = IKStatus::SUCCESS;
    result.joint_state.theta1 = theta1;
    result.joint_state.theta2 = theta2;

    return result;
}

double Manipulator2D::computeJacobianDeterminant(const JointState& state) const noexcept {
    return l1_ * l2_ * std::sin(state.theta2);
}

bool Manipulator2D::isSingular(const JointState& state, double tolerance) const noexcept {
    return std::abs(computeJacobianDeterminant(state)) < tolerance;
}

} // namespace kinematics
