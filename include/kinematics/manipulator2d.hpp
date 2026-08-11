#ifndef KINEMATICS_MANIPULATOR2D_HPP
#define KINEMATICS_MANIPULATOR2D_HPP

#include <cmath>
#include <stdexcept>

namespace kinematics {

struct JointState {
    double theta1{0.0};
    double theta2{0.0};
};

struct EndEffectorPose {
    double x{0.0};
    double y{0.0};
};

enum class IKStatus {
    SUCCESS,
    OUT_OF_REACH,
    SINGULARITY
};

enum class ElbowConfig {
    ELBOW_UP,   // theta2 >= 0
    ELBOW_DOWN  // theta2 <= 0
};

struct IKResult {
    IKStatus status{IKStatus::SUCCESS};
    JointState joint_state{0.0, 0.0};
};

class Manipulator2D {
public:
    Manipulator2D(double l1, double l2);

    // Getters for link lengths
    double getL1() const noexcept { return l1_; }
    double getL2() const noexcept { return l2_; }

    // Forward Kinematics
    EndEffectorPose computeFK(const JointState& state) const noexcept;
    EndEffectorPose computeFK(double theta1, double theta2) const noexcept;

    // Inverse Kinematics
    IKResult computeIK(const EndEffectorPose& pose, ElbowConfig config = ElbowConfig::ELBOW_DOWN) const noexcept;
    IKResult computeIK(double x, double y, ElbowConfig config = ElbowConfig::ELBOW_DOWN) const noexcept;

    // Kinematic analysis utilities
    double computeJacobianDeterminant(const JointState& state) const noexcept;
    bool isSingular(const JointState& state, double tolerance = 1e-6) const noexcept;

private:
    double l1_;
    double l2_;
};

} // namespace kinematics

#endif // KINEMATICS_MANIPULATOR2D_HPP
