#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
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
      .def_readwrite("cart_pos", &toppra3::TimedWaypoint::cart_pos)
      .def_readwrite("cart_vel", &toppra3::TimedWaypoint::cart_vel)
      .def_readwrite("cart_acc", &toppra3::TimedWaypoint::cart_acc)
      .def_readwrite("frame_name", &toppra3::TimedWaypoint::frame_name)
      .def_readwrite("time_from_start",
                     &toppra3::TimedWaypoint::time_from_start)
      .def_readwrite("segment_index", &toppra3::TimedWaypoint::segment_index)
      .def_readwrite("is_path_position",
                     &toppra3::TimedWaypoint::is_path_position)
      .def_readwrite("time_from_previous",
                     &toppra3::TimedWaypoint::time_from_previous);

  // Bind InputData class with all parameters
  py::class_<toppra3::InputData>(m, "InputData")
      .def(py::init<int,                  // num_joints
                    std::vector<double>,  // global_max_joint_velocity
                    std::vector<double>,  // global_max_joint_acceleration
                    std::vector<double>,  // global_max_joint_jerk
                    std::vector<double>,  // global_max_joint_torque
                    std::vector<int>,     // segment_indices
                    std::vector<std::vector<double>>,  // waypoint_scale_factors
                    std::vector<double>,  // waypoint_max_cart_vel_mm_per_s
                    std::vector<double>,  // waypoint_max_cart_acc_mm_per_s2
                    std::string,          // frame_name
                    std::vector<std::vector<double>>  // waypoints
                    >(),
           py::arg("num_joints"), py::arg("global_max_joint_velocity"),
           py::arg("global_max_joint_acceleration"),
           py::arg("global_max_joint_jerk"), py::arg("global_max_joint_torque"),
           py::arg("segment_indices"), py::arg("waypoint_scale_factors"),
           py::arg("waypoint_max_cart_vel_mm_per_s"),
           py::arg("waypoint_max_cart_acc_mm_per_s2"), py::arg("frame_name"),
           py::arg("waypoints"))
      .def_readwrite("num_joints_", &toppra3::InputData::num_joints_)
      .def_readwrite("global_max_joint_velocity",
                     &toppra3::InputData::global_max_joint_velocity)
      .def_readwrite("global_max_joint_acceleration",
                     &toppra3::InputData::global_max_joint_acceleration)
      .def_readwrite("global_max_joint_jerk",
                     &toppra3::InputData::global_max_joint_jerk)
      .def_readwrite("global_max_joint_torque",
                     &toppra3::InputData::global_max_joint_torque)
      .def_readwrite("segment_indices", &toppra3::InputData::segment_indices)
      .def_readwrite("waypoint_scale_factors",
                     &toppra3::InputData::waypoint_scale_factors)
      .def_readwrite("waypoint_max_cart_vel_mm_per_s",
                     &toppra3::InputData::waypoint_max_cart_vel_mm_per_s)
      .def_readwrite("waypoint_max_cart_acc_mm_per_s2",
                     &toppra3::InputData::waypoint_max_cart_acc_mm_per_s2)
      .def_readwrite("frame_name", &toppra3::InputData::frame_name)
      .def_readwrite("waypoints", &toppra3::InputData::waypoints);

  py::class_<toppra3::OutputData>(m, "OutputData")
      .def(py::init<>())
      .def_readwrite("waypoints", &toppra3::OutputData::waypoints)
      .def_readwrite("success", &toppra3::OutputData::success);

  py::class_<toppra3::Toppra3Parameterization>(m, "Toppra3Parameterization")
      .def(py::init<int, std::string>())
      .def("solve", &toppra3::Toppra3Parameterization::solve,
           py::arg("input_data"), py::arg("use_jerk_limits") = true,
           py::call_guard<py::gil_scoped_release>())
      .def("get_num_joints", &toppra3::Toppra3Parameterization::getNumJoints);
}
