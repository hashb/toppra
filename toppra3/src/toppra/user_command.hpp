#pragma once

#include <Eigen/Dense>
#include <iostream>

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

    lvm.resize(n);
    lam.resize(n);
  }
  int getsize() { return q.size(); }
};
