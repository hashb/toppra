#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "toppra3.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_toppra3, m) {
    m.doc() = "Python bindings for TOPPRA3 trajectory parameterization library";

    // Bind TimedWaypoint struct
    py::class_<toppra3::TimedWaypoint>(m, "TimedWaypoint")
        .def(py::init<>())
        .def_readwrite("q", &toppra3::TimedWaypoint::q)
        .def_readwrite("dq", &toppra3::TimedWaypoint::dq)
        .def_readwrite("ddq", &toppra3::TimedWaypoint::ddq)
        .def_readwrite("time_from_start", &toppra3::TimedWaypoint::time_from_start)
        .def_readwrite("segment_index", &toppra3::TimedWaypoint::segment_index)
        .def_readwrite("is_path_position", &toppra3::TimedWaypoint::is_path_position)
        .def_readwrite("time_from_previous", &toppra3::TimedWaypoint::time_from_previous);

    // Bind InputData class with all parameters
    py::class_<toppra3::InputData>(m, "InputData")
        .def(py::init<int, std::vector<double>, std::vector<double>, 
                     std::vector<double>, std::vector<int>,
                     std::vector<std::vector<double>>,
                     std::vector<std::vector<double>>>(),
             py::arg("num_joints"),
             py::arg("max_joint_velocity"),
             py::arg("max_joint_acceleration"), 
             py::arg("max_joint_jerk"),
             py::arg("segment_indices"),
             py::arg("scale_factors"),
             py::arg("waypoints"))
        .def_readwrite("num_joints_", &toppra3::InputData::num_joints_)
        .def_readwrite("max_joint_velocity", &toppra3::InputData::max_joint_velocity)
        .def_readwrite("max_joint_acceleration", &toppra3::InputData::max_joint_acceleration)
        .def_readwrite("max_joint_jerk", &toppra3::InputData::max_joint_jerk)
        .def_readwrite("segment_indices", &toppra3::InputData::segment_indices)
        .def_readwrite("scale_factors", &toppra3::InputData::scale_factors)
        .def_readwrite("waypoints", &toppra3::InputData::waypoints);

    py::class_<toppra3::OutputData>(m, "OutputData")
        .def(py::init<>())
        .def_readwrite("waypoints", &toppra3::OutputData::waypoints)
        .def_readwrite("success", &toppra3::OutputData::success);

    py::class_<toppra3::Toppra3Parameterization>(m, "Toppra3Parameterization")
        .def(py::init<int>())
        .def("solve", &toppra3::Toppra3Parameterization::solve,
             py::arg("input_data"),
             py::arg("use_jerk_limits") = true)
        .def("get_num_joints", &toppra3::Toppra3Parameterization::getNumJoints);
}
