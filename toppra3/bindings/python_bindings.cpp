#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "toppra3.hpp"

namespace py = pybind11;

PYBIND11_MODULE(toppra3, m) {
    m.doc() = "Python bindings for TOPPRA3 trajectory parameterization library";

    py::class_<RobotLimits>(m, "RobotLimits")
        .def(py::init<int>())
        .def_readwrite("max_joint_velocity", &RobotLimits::max_joint_velocity)
        .def_readwrite("max_joint_acceleration", &RobotLimits::max_joint_acceleration)
        .def_readwrite("max_joint_jerk", &RobotLimits::max_joint_jerk)
        .def_readwrite("max_joint_torque", &RobotLimits::max_joint_torque)
        .def_readwrite("max_linear_velocity", &RobotLimits::max_linear_velocity)
        .def_readwrite("max_linear_acceleration", &RobotLimits::max_linear_acceleration);

    py::class_<Toppra3Parameterization>(m, "Toppra3Parameterization")
        .def(py::init<int>())
        .def("solve", &Toppra3Parameterization::solve,
             py::arg("waypoints"),
             py::arg("limits"),
             py::arg("use_jerk_limits") = true)
        .def("get_state", [](Toppra3Parameterization& self, double time) {
            Eigen::VectorXd position, velocity, acceleration;
            position.resize(self.num_joints_);
            velocity.resize(self.num_joints_);
            acceleration.resize(self.num_joints_);
            
            self.getState(time, position, velocity, acceleration);
            
            return py::make_tuple(position, velocity, acceleration);
        }, "Get trajectory state (position, velocity, acceleration) at given time")
        .def("get_duration", &Toppra3Parameterization::getDuration);
} 