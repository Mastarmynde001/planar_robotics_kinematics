#include "kinematics/manipulator2d.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>
#include <stdexcept>

namespace py = pybind11;
using namespace kinematics;

PYBIND11_MODULE(kinematics_core_py, m) {
    m.doc() = "Python bindings for 2-DOF Planar Robot Manipulator Kinematics Core";

    // Bind IKStatus enum
    py::enum_<IKStatus>(m, "IKStatus")
        .value("SUCCESS", IKStatus::SUCCESS)
        .value("OUT_OF_REACH", IKStatus::OUT_OF_REACH)
        .value("SINGULARITY", IKStatus::SINGULARITY)
        .export_values();

    // Bind ElbowConfig enum
    py::enum_<ElbowConfig>(m, "ElbowConfig")
        .value("ELBOW_UP", ElbowConfig::ELBOW_UP)
        .value("ELBOW_DOWN", ElbowConfig::ELBOW_DOWN)
        .export_values();

    // Bind JointState struct
    py::class_<JointState>(m, "JointState")
        .def(py::init<double, double>(), py::arg("theta1") = 0.0, py::arg("theta2") = 0.0)
        .def_readwrite("theta1", &JointState::theta1)
        .def_readwrite("theta2", &JointState::theta2)
        .def("__repr__", [](const JointState& js) {
            std::ostringstream os;
            os << "JointState(theta1=" << js.theta1 << ", theta2=" << js.theta2 << ")";
            return os.str();
        })
        .def("__len__", [](const JointState&) { return 2; })
        .def("__getitem__", [](const JointState& js, size_t idx) {
            if (idx == 0) return js.theta1;
            if (idx == 1) return js.theta2;
            throw py::index_error("JointState index out of range (must be 0 or 1)");
        });

    // Bind EndEffectorPose struct
    py::class_<EndEffectorPose>(m, "EndEffectorPose")
        .def(py::init<double, double>(), py::arg("x") = 0.0, py::arg("y") = 0.0)
        .def_readwrite("x", &EndEffectorPose::x)
        .def_readwrite("y", &EndEffectorPose::y)
        .def("__repr__", [](const EndEffectorPose& pose) {
            std::ostringstream os;
            os << "EndEffectorPose(x=" << pose.x << ", y=" << pose.y << ")";
            return os.str();
        })
        .def("__len__", [](const EndEffectorPose&) { return 2; })
        .def("__getitem__", [](const EndEffectorPose& pose, size_t idx) {
            if (idx == 0) return pose.x;
            if (idx == 1) return pose.y;
            throw py::index_error("EndEffectorPose index out of range (must be 0 or 1)");
        });

    // Bind IKResult struct
    py::class_<IKResult>(m, "IKResult")
        .def_readwrite("status", &IKResult::status)
        .def_readwrite("joint_state", &IKResult::joint_state)
        .def("__repr__", [](const IKResult& res) {
            std::ostringstream os;
            os << "IKResult(status=" << static_cast<int>(res.status)
               << ", joint_state=JointState(" << res.joint_state.theta1 << ", " << res.joint_state.theta2 << "))";
            return os.str();
        });

    // Bind Manipulator2D class
    py::class_<Manipulator2D>(m, "Manipulator2D")
        .def(py::init<double, double>(), py::arg("l1"), py::arg("l2"))
        .def_property_readonly("l1", &Manipulator2D::getL1)
        .def_property_readonly("l2", &Manipulator2D::getL2)

        // computeFK overloads
        .def("computeFK", [](const Manipulator2D& self, double theta1, double theta2) {
            return self.computeFK(theta1, theta2);
        }, py::arg("theta1"), py::arg("theta2"), "Compute Forward Kinematics from theta1 and theta2.")
        .def("computeFK", [](const Manipulator2D& self, const JointState& state) {
            return self.computeFK(state);
        }, py::arg("state"), "Compute Forward Kinematics from a JointState.")

        // computeIK overloads accepting (x, y, elbow_up) and (pose, elbow_up)
        .def("computeIK", [](const Manipulator2D& self, double x, double y, bool elbow_up) {
            if (!std::isfinite(x) || !std::isfinite(y)) {
                throw py::value_error("Target coordinates x and y must be finite real numbers.");
            }
            ElbowConfig config = elbow_up ? ElbowConfig::ELBOW_UP : ElbowConfig::ELBOW_DOWN;
            IKResult res = self.computeIK(x, y, config);
            if (res.status == IKStatus::OUT_OF_REACH) {
                throw py::value_error("Target position is out of reach.");
            }
            if (res.status == IKStatus::SINGULARITY) {
                throw py::value_error("Target position is at a kinematic singularity.");
            }
            return res.joint_state;
        }, py::arg("x"), py::arg("y"), py::arg("elbow_up") = false,
           "Compute Inverse Kinematics for target (x, y) with optional elbow_up flag. Throws ValueError on failure.")
        .def("computeIK", [](const Manipulator2D& self, const EndEffectorPose& pose, bool elbow_up) {
            if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) {
                throw py::value_error("Target coordinates x and y must be finite real numbers.");
            }
            ElbowConfig config = elbow_up ? ElbowConfig::ELBOW_UP : ElbowConfig::ELBOW_DOWN;
            IKResult res = self.computeIK(pose, config);
            if (res.status == IKStatus::OUT_OF_REACH) {
                throw py::value_error("Target position is out of reach.");
            }
            if (res.status == IKStatus::SINGULARITY) {
                throw py::value_error("Target position is at a kinematic singularity.");
            }
            return res.joint_state;
        }, py::arg("pose"), py::arg("elbow_up") = false,
           "Compute Inverse Kinematics for target pose with optional elbow_up flag. Throws ValueError on failure.")

        // computeIK_raw returning full IKResult without throwing exception
        .def("computeIK_raw", [](const Manipulator2D& self, double x, double y, bool elbow_up) {
            ElbowConfig config = elbow_up ? ElbowConfig::ELBOW_UP : ElbowConfig::ELBOW_DOWN;
            return self.computeIK(x, y, config);
        }, py::arg("x"), py::arg("y"), py::arg("elbow_up") = false)

        // Utility methods
        .def("computeJacobianDeterminant", [](const Manipulator2D& self, const JointState& state) {
            return self.computeJacobianDeterminant(state);
        }, py::arg("state"))
        .def("computeJacobianDeterminant", [](const Manipulator2D& self, double theta1, double theta2) {
            return self.computeJacobianDeterminant(JointState{theta1, theta2});
        }, py::arg("theta1"), py::arg("theta2"))
        .def("isSingular", [](const Manipulator2D& self, const JointState& state, double tol) {
            return self.isSingular(state, tol);
        }, py::arg("state"), py::arg("tolerance") = 1e-6)
        .def("isSingular", [](const Manipulator2D& self, double theta1, double theta2, double tol) {
            return self.isSingular(JointState{theta1, theta2}, tol);
        }, py::arg("theta1"), py::arg("theta2"), py::arg("tolerance") = 1e-6);
}
