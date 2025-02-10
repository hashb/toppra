#pragma once

#include <iostream>

#include <Eigen/Dense>
#include <rossy_utils/io/io_utilities.hpp>

class PLANNING_COMMAND {
 public:
  PLANNING_COMMAND() {
    joint_path.clear();

    // should be set
    max_joint_acceleration = {};
    max_joint_speed = {};
    max_joint_jerk = {};

    max_linear_acceleration = -1.;
    max_linear_speed = -1.;

    acc_percentage_path_ratio = 0.;
    acc_percentage = 1.;
    dec_percentage_path_ratio = 0.;
    dec_percentage = 1.;
  }
  ~PLANNING_COMMAND() {}

 public:
  std::vector<Eigen::VectorXd> joint_path;

  Eigen::VectorXd max_joint_acceleration;
  Eigen::VectorXd max_joint_speed;
  Eigen::VectorXd max_joint_jerk;

  double max_linear_acceleration;
  double max_linear_speed;

  double acc_percentage_path_ratio;
  double acc_percentage;
  double dec_percentage_path_ratio;
  double dec_percentage;
};

class SYSTEM_DATA {
 public:
  std::vector<double> s;
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::VectorXd> dq;   // q'
  std::vector<Eigen::VectorXd> ddq;  // q''

  std::vector<Eigen::VectorXd> av;   // vel coeffs
  std::vector<Eigen::VectorXd> vm2;  // vel max square
  std::vector<Eigen::VectorXd> am;   // acceleration max
  std::vector<Eigen::VectorXd> jm;   // jerk max

  SYSTEM_DATA() { resize(0); }
  ~SYSTEM_DATA() {}
  void resize(int n) {
    s.resize(n);
    q.resize(n);
    dq.resize(n);
    ddq.resize(n);

    am.resize(n);
    jm.resize(n);
    av.resize(n);
    vm2.resize(n);

  }
  int getsize() { return q.size(); }
  void savedata() {
    for (auto& data : s) rossy_utils::saveValue(data, "pathparam/s");
    for (auto& data : q) rossy_utils::saveVector(data, "pathparam/q");
    for (auto& data : dq) rossy_utils::saveVector(data, "pathparam/dq");
    for (auto& data : ddq) rossy_utils::saveVector(data, "pathparam/ddq");
    for (auto& data : am) rossy_utils::saveVector(data, "pathparam/am");
    for (auto& data : jm) rossy_utils::saveVector(data, "pathparam/jm");
    for (auto& data : av) rossy_utils::saveVector(data, "pathparam/av");
    for (auto& data : vm2) rossy_utils::saveVector(data, "pathparam/vm2");
  }
};
