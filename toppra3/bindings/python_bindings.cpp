#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "toppra3.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_toppra3, m) {
    m.doc() = "Python bindings for TOPPRA3 trajectory parameterization library";

    py::class_<toppra3::RobotLimits>(m, "RobotLimits")
        .def(py::init<int>())
        .def("get_max_joint_velocity", [](const toppra3::RobotLimits& self) {
            return Eigen::VectorXd(self.max_joint_velocity);
        })
        .def("set_max_joint_velocity", [](toppra3::RobotLimits& self, const Eigen::VectorXd& value) {
            self.max_joint_velocity = value;
        })
        .def("get_max_joint_acceleration", [](const toppra3::RobotLimits& self) {
            return Eigen::VectorXd(self.max_joint_acceleration);
        })
        .def("set_max_joint_acceleration", [](toppra3::RobotLimits& self, const Eigen::VectorXd& value) {
            self.max_joint_acceleration = value;
        })
        .def("get_max_joint_jerk", [](const toppra3::RobotLimits& self) {
            return Eigen::VectorXd(self.max_joint_jerk);
        })
        .def("set_max_joint_jerk", [](toppra3::RobotLimits& self, const Eigen::VectorXd& value) {
            self.max_joint_jerk = value;
        })
        .def("get_max_joint_torque", [](const toppra3::RobotLimits& self) {
            return Eigen::VectorXd(self.max_joint_torque);
        })
        .def("set_max_joint_torque", [](toppra3::RobotLimits& self, const Eigen::VectorXd& value) {
            self.max_joint_torque = value;
        });

    py::class_<toppra3::Toppra3Parameterization>(m, "Toppra3Parameterization")
        .def(py::init<int>())
        .def("solve", &toppra3::Toppra3Parameterization::solve,
             py::arg("waypoints"),
             py::arg("limits"),
             py::arg("use_jerk_limits") = true);
}
