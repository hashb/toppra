#pragma once

#include <iostream>

#include <Eigen/Dense>
#include <rossy_utils/io/io_utilities.hpp>

class TRAJ_DATA {
 public:
  TRAJ_DATA() {
    tdata.clear();
    qdata.clear();
    dqdata.clear();
    xdata.clear();
    dxdata.clear();
    period = 0.01;
    singularityinpath = false;
  }
  ~TRAJ_DATA() {}

 public:
  std::vector<double> tdata;
  std::vector<Eigen::VectorXd> qdata;
  std::vector<Eigen::VectorXd> dqdata;
  std::vector<Eigen::VectorXd> xdata;
  std::vector<Eigen::VectorXd> dxdata;
  double period;
  bool singularityinpath;
};

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

class WPT_DATA {
 public:
  WPT_DATA() {
    data.clear();
  }
  ~WPT_DATA() {}

  int getsize() { return data.size(); }
  Eigen::VectorXd getdata(int i) {
    if (i > 0 && i < data.size())
      return data[i];
    else
      return Eigen::VectorXd::Zero(0);
  }

 public:
  std::vector<Eigen::VectorXd> data;
};

class VEC_DATA {
 public:
  Eigen::VectorXd data;
  VEC_DATA() {}
  ~VEC_DATA() {}
};

class SYSTEM_DATA {
 public:
  std::vector<double> s;
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::VectorXd> dq;   // q'
  std::vector<Eigen::VectorXd> ddq;  // q''

  std::vector<Eigen::VectorXd> m;    // mass coeffs
  std::vector<Eigen::VectorXd> b;    // coriolis coeffs
  std::vector<Eigen::VectorXd> g;    // gravity coeffs
  std::vector<Eigen::VectorXd> av;   // vel coeffs
  std::vector<Eigen::VectorXd> vm2;  // vel max square
  std::vector<Eigen::VectorXd> tm;   // torque max
  std::vector<Eigen::VectorXd> am;   // acceleration max
  std::vector<Eigen::VectorXd> jm;   // jerk max

  // linear constraints: bodynodejacobian
  // v = (Jq')*ds2
  // a = (J'q'+Jq'')*ds2 + (Jq')*dds
  std::vector<Eigen::VectorXd> ee;  // ee pos
  std::vector<Eigen::VectorXd> ee_v;
  std::vector<Eigen::VectorXd> ee_a;

  // for grasping constraints: bodynodebodyjacob
  std::vector<Eigen::VectorXd> ee_w;
  std::vector<Eigen::VectorXd> ee_aw1;
  std::vector<Eigen::VectorXd> ee_aw2;
  std::vector<Eigen::VectorXd> ee_av1;
  std::vector<Eigen::VectorXd> ee_av2;
  std::vector<Eigen::VectorXd> ee_grav;

  // a_g s'' + b_g s'2 + c_g < 0
  std::vector<Eigen::VectorXd> a_g;
  std::vector<Eigen::VectorXd> b_g;
  std::vector<Eigen::VectorXd> c_g;

  // Fi = m_suc s'' + b_suc s'2 + g_suc
  std::vector<Eigen::VectorXd> m_suc;
  std::vector<Eigen::VectorXd> b_suc;
  std::vector<Eigen::VectorXd> g_suc;

  // linear vel/acc limits : -1 if not activated
  std::vector<double> lvm;
  std::vector<double> lam;

  SYSTEM_DATA() { resize(0); }
  ~SYSTEM_DATA() {}
  void resize(int n) {
    s.resize(n);
    q.resize(n);
    dq.resize(n);
    ddq.resize(n);

    m.resize(n);
    b.resize(n);
    g.resize(n);
    tm.resize(n);
    am.resize(n);
    jm.resize(n);
    av.resize(n);
    vm2.resize(n);

    ee.resize(n);
    ee_v.resize(n);
    ee_a.resize(n);

    ee_w.resize(n);
    ee_aw1.resize(n);
    ee_aw2.resize(n);
    ee_av1.resize(n);
    ee_av2.resize(n);
    ee_grav.resize(n);

    a_g.clear();
    b_g.clear();
    c_g.clear();

    m_suc.clear();
    b_suc.clear();
    g_suc.clear();

    lvm.resize(n);
    lam.resize(n);
  }
  int getsize() { return q.size(); }
  void savedata() {
    for (auto& data : s) rossy_utils::saveValue(data, "pathparam/s");
    for (auto& data : q) rossy_utils::saveVector(data, "pathparam/q");
    for (auto& data : dq) rossy_utils::saveVector(data, "pathparam/dq");
    for (auto& data : ddq) rossy_utils::saveVector(data, "pathparam/ddq");
    for (auto& data : m) rossy_utils::saveVector(data, "pathparam/m");
    for (auto& data : b) rossy_utils::saveVector(data, "pathparam/b");
    for (auto& data : g) rossy_utils::saveVector(data, "pathparam/g");
    for (auto& data : tm) rossy_utils::saveVector(data, "pathparam/tm");
    for (auto& data : am) rossy_utils::saveVector(data, "pathparam/am");
    for (auto& data : jm) rossy_utils::saveVector(data, "pathparam/jm");
    for (auto& data : av) rossy_utils::saveVector(data, "pathparam/av");
    for (auto& data : vm2) rossy_utils::saveVector(data, "pathparam/vm2");
    for (auto& data : ee) rossy_utils::saveVector(data, "pathparam/ee");
    for (auto& data : ee_v) rossy_utils::saveVector(data, "pathparam/ee_v");
    for (auto& data : ee_a) rossy_utils::saveVector(data, "pathparam/ee_a");
    for (auto& data : ee_w) rossy_utils::saveVector(data, "pathparam/ee_w");
    for (auto& data : ee_aw1) rossy_utils::saveVector(data, "pathparam/ee_aw1");
    for (auto& data : ee_aw2) rossy_utils::saveVector(data, "pathparam/ee_aw2");
    for (auto& data : ee_av1) rossy_utils::saveVector(data, "pathparam/ee_av1");
    for (auto& data : ee_av2) rossy_utils::saveVector(data, "pathparam/ee_av2");
    for (auto& data : ee_grav)
      rossy_utils::saveVector(data, "pathparam/ee_grav");
    for (auto& data : lvm) rossy_utils::saveValue(data, "pathparam/lvm");
    for (auto& data : lam) rossy_utils::saveValue(data, "pathparam/lam");
  }
};
