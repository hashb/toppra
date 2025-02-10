#include "toppra/topt_solver.hpp"

#ifndef TOPT_DEBUG_PRINT
#define TOPT_DEBUG_PRINT 0
#endif

#define TOPT_DEBUG_MSG(msg) do { \
    if (TOPT_DEBUG_PRINT) { \
        std::cout << msg; \
    } \
} while(0)

#include "clp/clpwrapper.hpp"
#include "toppra/topt_utils.hpp"
#include "toppra/trajectory_manager.hpp"
#include "rossy_utils/general/clock.hpp"
#include "rossy_utils/math/math_utilities.hpp"

ToptSolver::ToptSolver(int dim) {
  // rossy_utils::pretty_constructor(2, "ToptSolver");
  dim_ = dim;  // robot actuator dimension
  lpsolver_ = new LPSolver();

  clock_ = new Clock();
  initializeDimensions();
}

void ToptSolver::initializeDimensions() {
  //
  dimIneq1_ = dim_ + 1;
  //
  dimIneq2_ = 4 * dim_;
  //
  // dimIneq3_ = 2*dim_+1; // if spline condition added
  dimIneq3_ = 2 * dim_ + 2;  // if acc jump limit added
}

void ToptSolver::get1stlimit(int k, InequalData& constraints) {
  //  A*x[k] < b
  // -1*x[k] < -MIN_SDOT_
  // dimIneq1_ = dim_ +1
  Eigen::VectorXd A, b;
  A = rossy_utils::vStack(sysdata_.av[k], -1.0);
  b = rossy_utils::vStack(sysdata_.vm2[k], -MIN_SDOT_);

  constraints.A = A;
  constraints.b = b;
}

void ToptSolver::get2ndlimit(int k, InequalData& constraints) {
  // A*[ x[k]; x[k+1] ] < b
  // dimIneq2_ = 4*dim_(trq) + 4*dim_(acc)
  constraints.A = Eigen::MatrixXd::Zero(4 * dim_, 2);
  constraints.b = Eigen::VectorXd::Zero(4 * dim_);

  // acceleration limit
  // 1) -am[k] <
  //  dq[k]/2ds[k] x[k+1] + (ddq[k]-dq[k]/2ds) x[k]
  //                                        < am[k]
  Eigen::VectorXd ak1 = sysdata_.dq[k] / 2. / (sysdata_.s[k + 1] - sysdata_.s[k]);
  Eigen::VectorXd ak0 = sysdata_.ddq[k] - ak1;
  constraints.A.block(0 * dim_, 0, dim_, 1) = ak0;
  constraints.A.block(0 * dim_, 1, dim_, 1) = ak1;
  constraints.b.segment(0 * dim_, dim_) = sysdata_.am[k];
  constraints.A.block(1 * dim_, 0, dim_, 1) = -ak0;
  constraints.A.block(1 * dim_, 1, dim_, 1) = -ak1;
  constraints.b.segment(1 * dim_, dim_) = sysdata_.am[k];
  // 2) -am[k+1] <
  // ( ddq[k+1] + dq[k+1]/2ds[k] ) x[k+1] - (dq[k+1]/2ds) x[k]
  //                                        < am[k+1]
  ak0 = -sysdata_.dq[k + 1] / 2. / (sysdata_.s[k + 1] - sysdata_.s[k]);
  ak1 = sysdata_.ddq[k + 1] - ak0;
  constraints.A.block(2 * dim_, 0, dim_, 1) = ak0;
  constraints.A.block(2 * dim_, 1, dim_, 1) = ak1;
  constraints.b.segment(2 * dim_, dim_) = sysdata_.am[k + 1];
  constraints.A.block(3 * dim_, 0, dim_, 1) = -ak0;
  constraints.A.block(3 * dim_, 1, dim_, 1) = -ak1;
  constraints.b.segment(3 * dim_, dim_) = sysdata_.am[k + 1];
}

void ToptSolver::get3rdlimit(int k, const std::vector<double>& x0_list,
                             InequalData& constraints) {
  // dddq[k] = (ddq[k+1] - ddq[k]) / dt[k]
  // ddq[k+1] - ddq[k] = J0 x[k] + J1 x[k+1] + J2 x[k+2]
  Eigen::VectorXd J0, J1, J2;
  double dsk = sysdata_.s[k + 1] - sysdata_.s[k];
  double dskk = sysdata_.s[k + 2] - sysdata_.s[k + 1];
  // std::cout<<"dsk="<<dsk<<", dskk="<<dskk<<std::endl;
  J0 = sysdata_.dq[k] / 2. / dsk - sysdata_.ddq[k];
  J1 = -sysdata_.dq[k + 1] / 2. / dskk - sysdata_.dq[k] / 2. / dsk +
       sysdata_.ddq[k + 1];
  J2 = sysdata_.dq[k + 1] / 2. / dskk;
  // std::cout<<"J0="<<J0.transpose()<<", J1="<<J1.transpose()
  //             <<", J2="<<J2.transpose()<<std::endl;

  // dt[k] = h(xbar, k) > hbar + dh0*x[k] + dh1*x[k+1] + dh2*x[k+2]
  double vk0, vk1, vk2;
  vk0 = sqrt(std::max(MIN_SDOT_, x0_list[k]));
  vk1 = sqrt(std::max(MIN_SDOT_, x0_list[k + 1]));
  vk2 = sqrt(std::max(MIN_SDOT_, x0_list[k + 2]));

  // need to check if vk0, vk1, vk2 are zero or not
  double hbar, dh0, dh1, dh2;
  if (k == 0) {
    dh0 = 0.;
    dh1 =
        (-dsk / (vk0 + vk1) / (vk0 + vk1) - dskk / (vk1 + vk2) / (vk1 + vk2)) *
        0.5 / vk1;
    dh2 = -dskk / (vk1 + vk2) / (vk1 + vk2) * 0.5 / vk2;
  } else if (k == x0_list.size() - 3) {
    dh0 = -dsk / (vk0 + vk1) / (vk0 + vk1) * 0.5 / vk0;
    dh1 =
        -(dsk / (vk0 + vk1) / (vk0 + vk1) + dskk / (vk1 + vk2) / (vk1 + vk2)) *
        0.5 / vk1;
    dh2 = 0.;
  } else {
    dh0 = -dsk / (vk0 + vk1) / (vk0 + vk1) * 0.5 / vk0;
    dh1 =
        -(dsk / (vk0 + vk1) / (vk0 + vk1) + dskk / (vk1 + vk2) / (vk1 + vk2)) *
        0.5 / vk1;
    dh2 = -dskk / (vk1 + vk2) / (vk1 + vk2) * 0.5 / vk2;
  }
  hbar = dsk / (vk0 + vk1) + dskk / (vk1 + vk2);
  // std::cout<<"hbar= "<< hbar <<", dh0="<<dh0<<", dh1="<<dh1
  //             <<", dh2="<<dh2<<std::endl;
  hbar = hbar - dh0 * x0_list[k] - dh1 * x0_list[k + 1] - dh2 * x0_list[k + 2];

  // add jerk condition
  constraints.A.resize(2 * J0.size(), 3);
  constraints.A.col(0) << -J0 - sysdata_.jm[k] * dh0, J0 - sysdata_.jm[k] * dh0;
  constraints.A.col(1) << -J1 - sysdata_.jm[k] * dh1, J1 - sysdata_.jm[k] * dh1;
  constraints.A.col(2) << -J2 - sysdata_.jm[k] * dh2, J2 - sysdata_.jm[k] * dh2;

  // std::cout<<constraints.A<<std::endl;

  Eigen::VectorXd temp;
  temp = sysdata_.jm[k] * hbar;
  constraints.b = rossy_utils::vStack(temp, temp);

  // std::cout<<constraints.b<<std::endl;

  // add spline condition
  // (-1/3-2r)x[k]+(-5/3-2/3*dsk/dskk-2(1-r))x[k+1]+2/3*(dsk/dskk)x[k+2] < 0
  // Eigen::MatrixXd Asp = Eigen::MatrixXd::Zero(1,3);
  //  Asp << (-1./3.), (-5./3.-2./3.*dsk/dskk), 2./3.*dsk/dskk;
  // constraints.A = rossy_utils::vStack(constraints.A, Asp);
  // constraints.b = rossy_utils::vStack(constraints.b, 0.);

  // add
  // -J < q'[k+1]/tstep * ( (x[k+2]-x[k+1])/dskk  - (x[k+1]-x[k])/dsk ) < J
  // -J/q'[k+1]*tstep < (1/dskk)*x[k+2] - (1/dskk+1/dsk)*x[k+1] + (1/dsk)*x[k]
  Eigen::MatrixXd Asp = Eigen::MatrixXd::Zero(2, 3);
  Asp << -1. / dsk, (1. / dskk + 1. / dsk), -(1. / dskk), 1. / dsk,
      -(1. / dskk + 1. / dsk), (1. / dskk);
  double dqdj =
      rossy_utils::getMaxRatioValue(sysdata_.dq[k + 1], sysdata_.jm[k]);
  double tstep = 0.02;
  Eigen::VectorXd bsp{{tstep / dqdj, tstep / dqdj}};
  constraints.A = rossy_utils::vStack(constraints.A, Asp);
  constraints.b = rossy_utils::vStack(constraints.b, bsp);
}

double ToptSolver::getFeasibleX(int k) {
  // given x[k+1]
  // min -x[k]
  // s.t. ax*x[k] < b
  Eigen::VectorXd ax, b;
  InequalData constraints;

  // add velocity constraints: A[0]*x[k] < b
  get1stlimit(k, constraints);
  ax = constraints.A.col(0);
  b = constraints.b;

  // solve LP
  double xmax_c_k;
  double ret = rossy_utils::linprog1d(-1, ax, b, xmax_c_k);
  return xmax_c_k;
}

double ToptSolver::getControllableX(int k, double xmax_c_kk) {
  // given: x[k+1] < xmax_c_kk
  // find x[k], x[k+1], min t
  // s.t. A0*x[k] + A1*x[k+1] < b

  // INPUT: k, xmax_c_kk
  // OUTPUT: xmax_c_k,
  double xmax_c_k;

  // add velocity constraints: A[0]*x[k] < b
  InequalData cst1;
  get1stlimit(k, cst1);

  // add torque limit constraints: A[0]*x[k] + A[1]*x[k+1] < b
  InequalData cst2;
  get2ndlimit(k, cst2);

  // case1 : if x_kk is fixed:
  if (xmax_c_kk < MIN_SDOT_) {
    Eigen::VectorXd ax = cst1.A.col(0);
    ax = rossy_utils::vStack(ax, cst2.A.col(0));
    Eigen::VectorXd b = cst1.b;
    b = rossy_utils::vStack(b, cst2.b - cst2.A.col(1) * xmax_c_kk);
    double ret = rossy_utils::linprog1d(-1, ax, b, xmax_c_k);
    return xmax_c_k;
  }

  // case2 : find x[k] that maximizes{ x[k] + x[k+1] }
  Eigen::MatrixXd Ax;
  Eigen::VectorXd b;
  Eigen::VectorXd ax = cst1.A.col(0);
  int colsize = cst1.A.rows();
  // 1nd limit: A[0]*x[k] + 0*x[k+1] < b
  Ax = rossy_utils::hStack(ax, Eigen::VectorXd::Zero(colsize));
  // 2nd limit: : A[0]*x[k] + A[1]*x[k+1] < b
  Ax = rossy_utils::vStack(Ax, cst2.A);
  b = rossy_utils::vStack(cst1.b, cst2.b);
  // bwd reachability:  x[k+1] < xmax_c_kk
  Eigen::MatrixXd Axtmp{{0., 1.}};
  Ax = rossy_utils::vStack(Ax, Axtmp);
  b = rossy_utils::vStack(b, xmax_c_kk);
  // solve LP
  Eigen::VectorXd f2d{{-1, -1}};
  // Eigen::VectorXd f2d {{-1, 0}};
  Eigen::VectorXd soln;
  double ret = rossy_utils::linprog2d(f2d, Ax, b, soln);

  // return soln[0];

  // if solution looks ok
  if (soln[0] > 2 * MIN_SDOT_ && soln[1] > 2 * MIN_SDOT_ &&
      soln[0] * 10. > soln[1] && soln[1] * 10. > soln[0]) {
    xmax_c_k = soln[0];
    xmax_c_kk = soln[1];
    return xmax_c_k;
  } else {
    std::cout << " HERE!!! solution not good @ " << k << std::endl;
    std::cout << "soln = " << soln[0] << ", " << soln[1] << std::endl;
    int N = 7;
    double xkk_tmp, xmaxk_tmp;
    double t(0.), mint(1000.);
    xmax_c_k = MIN_SDOT_;
    double given_xmax_c_kk = xmax_c_kk;
    for (int i(0); i < N; ++i) {
      xkk_tmp = (double)i / (double)(N - 1) * MIN_SDOT_ +
                (double)(N - 1 - i) / (double)(N - 1) * given_xmax_c_kk;

      Eigen::VectorXd ax = cst1.A.col(0);
      ax = rossy_utils::vStack(ax, cst2.A.col(0));
      Eigen::VectorXd b = cst1.b;
      b = rossy_utils::vStack(b, cst2.b - cst2.A.col(1) * xkk_tmp);
      ret = rossy_utils::linprog1d(-1, ax, b, xmaxk_tmp);
      t = 1 / sqrt(xkk_tmp) + 1 / sqrt(xmaxk_tmp);

      if (t < mint) {
        mint = t;
        xmax_c_kk = xkk_tmp;
        xmax_c_k = xmaxk_tmp;
      }
    }
    std::cout << "soln = " << xmax_c_k << ", " << xmax_c_kk << std::endl;
    return xmax_c_k;
  }
}

double ToptSolver::getReachableXMax(int k, double xmax_c_k,
                                    double xmax_r_kpre) {
  // given x[k-1]
  // min -x[k]
  // s.t. ax1*x[k] < b-ax0*x[k-1]
  Eigen::VectorXd ax, b;
  InequalData constraints;

  // add next state controllable
  get2ndlimit(k - 1, constraints);
  ax = constraints.A.col(1);
  b = constraints.b - constraints.A.col(0) * xmax_r_kpre;

  // add next state controllable
  ax = rossy_utils::vStack(ax, 1);
  b = rossy_utils::vStack(b, xmax_c_k);

  ax = rossy_utils::vStack(ax, -1);
  b = rossy_utils::vStack(b, -MIN_SDOT_);

  double xmax_r_k, ret;
  ret = rossy_utils::linprog1d(-1, ax, b, xmax_r_k);
  return xmax_r_k;
}

bool ToptSolver::solve(const SYSTEM_DATA& sysdata, TrajectoryManager* traj,
                       bool thirdorder) {
  // set system variables
  sysdata_ = sysdata;

  // 0. initialize sets containers
  initializeSets();
  initializeDimensions();

  // 1. compute controllable_xmax_, reachable_xmax_
  clock_->start();
  bool soln_exist = solveTOPPRA();
  TOPT_DEBUG_MSG("solveTOPPRA()=" << clock_->stop() << "ms" << std::endl);

  // return if no jerk limit violation
  double limitratio = 2.0;

  if (!soln_exist) {
    TOPT_DEBUG_MSG("------ toppra - 2nd order ------ " << std::endl);
    limitratio = checkLimits(reachable_xmax_);
    return false;
  }
  if (!thirdorder || limitratio < 1.0 + 1e-5) {
    traj->setT2SSpline(sysdata_.s, reachable_xmax_);
    return true;
  }

  // 2. compute trackable_xmax_
  clock_->start();
  solveTOPP3(reachable_xmax_);
  TOPT_DEBUG_MSG("solveTOPP3()=" << clock_->stop() << "ms" << std::endl);

  // 3. generate spline_t2s_
  clock_->start();
  traj->setT2SSpline(sysdata_.s, trackable_xmax_);
  TOPT_DEBUG_MSG("setT2QSpline()=" << clock_->stop() << "ms" << std::endl);

  return true;
}

void ToptSolver::resolve(double planning_time, TrajectoryManager* traj) {
  initializeDimensions();

  // 1. Find the next interval
  int idx = traj->evaluateTimeInterval(planning_time);
  int nwpts = sysdata_.getsize();
  TOPT_DEBUG_MSG(idx << "- th interval / " << nwpts << std::endl);

  if (idx + 1 < nwpts)
    idx++;
  else
    return;

  // 2. solve TOPPRA
  clock_->start();
  controllable_xmax_ = trackable_xmax_;
  reachable_xmax_ = trackable_xmax_;
  solveTOPPRA(idx);
  TOPT_DEBUG_MSG("solveTOPPRA()=" << clock_->stop() << "ms" << std::endl);

  // 3. solve TOPP3
  clock_->start();
  solveTOPP3(reachable_xmax_, idx);
  TOPT_DEBUG_MSG("solveTOPP3()=" << clock_->stop() << "ms" << std::endl);

  // 4. reset spline
  clock_->start();
  traj->setT2QSpline(sysdata_.s, trackable_xmax_);
  TOPT_DEBUG_MSG("setT2QSpline()=" << clock_->stop() << "ms" << std::endl);
}

void ToptSolver::initializeSets() {
  int num_wpts = sysdata_.q.size();

  controllable_xmax_.resize(num_wpts);
  reachable_xmax_.resize(num_wpts);
  trackable_xmax_.resize(num_wpts);
  std::fill(controllable_xmax_.begin(), controllable_xmax_.end(), 0.);
  std::fill(reachable_xmax_.begin(), reachable_xmax_.end(), 0.);
  std::fill(trackable_xmax_.begin(), trackable_xmax_.end(), 0.);
}

void ToptSolver::solveTOPPRA0() {
  // only consider 1st order constraints
  int num_wpts = sysdata_.q.size();
  TOPT_DEBUG_MSG("num_wpts=" << num_wpts << std::endl);
  //
  feasible_xmax_.resize(num_wpts);
  feasible_xmax_[num_wpts - 1] = 0.;
  feasible_xmax_[0] = 0.;
  //
  double xmax;
  TOPT_DEBUG_MSG("feasible_xmax_ (inverse order) = ");
  for (int i(num_wpts - 2); i > 0; --i) {
    xmax = getFeasibleX(i);
    feasible_xmax_[i] = xmax;
    if (TOPT_DEBUG_PRINT) {
      std::cout << xmax << ", ";
    }
  }
  TOPT_DEBUG_MSG(std::endl);
}

bool ToptSolver::solveTOPPRA(int i_c) {
  // solve for x[i_c+1]~x[num_wpts-2]
  // x[0],..., x[i_c], and x[num_wpts-1] are fixed
  int num_wpts = sysdata_.q.size();

  // backward
  // update controllable_xmax_ for i>i_c
  double xmax_c = 0.;
  for (int i(num_wpts - 2); i > i_c - 1; --i) {
    xmax_c = getControllableX(i, xmax_c);
    controllable_xmax_[i] = xmax_c;
  }

  // forward
  // update controllable_xmax_ for i>i_c
  double xmax_r = reachable_xmax_[i_c];
  for (int i(i_c + 1); i < num_wpts; ++i) {
    xmax_c = controllable_xmax_[i];
    xmax_r = getReachableXMax(i, xmax_c, xmax_r);
    reachable_xmax_[i] = xmax_r;
  }

  // check if solution exists
  // if x = 0 during the motion
  for (int i(i_c + 1); i < num_wpts - 1; ++i) {
    if (reachable_xmax_[i] < 0.5 * MIN_SDOT_) {
      TOPT_DEBUG_MSG("solveTOPPRA() -> no solution exists" << std::endl);
      return false;
    }
  }
  return true;
}

void ToptSolver::solveTOPP3(const std::vector<double>& x0_list, int i_c) {
  // jerk consider
  int num_wpts = sysdata_.q.size();
  TOPT_DEBUG_MSG(" solveTOPP3 : num_wpts=" << num_wpts << std::endl);

  // x[0],..., x[i_c], and x[num_wpts-1] are fixed
  int nh = num_wpts - 2 - i_c;  // x[i_c+1]~x[num_wpts-2 = i_c+nh]

  // initial iteration
  InequalDataList constraints;
  Eigen::VectorXd f, x;
  std::vector<double> xprev_list = x0_list;
  std::vector<double> xopt_list = x0_list;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(0, 0);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(0);
  double topt = estimateMotionPeriod(x0_list);
  double alpha = 10.;  // trust region constant
  constraints = buildTOPP3Ineq(xprev_list, i_c, nh, alpha);
  for (auto& cst : constraints) {
    A = rossy_utils::vStack(A, cst.A);
    b = rossy_utils::vStack(b, cst.b);
  }
  // f = Eigen::VectorXd::Constant(nh, -1);
  f = getCostCoeffs(x0_list, i_c, nh);
  double ret = lpsolver_->solve(f, A, b, x);

  // update xnew
  double x_diff(0.), x_norm(0.), t_diff(1.);
  for (int k(1); k < nh + 1; k++) {
    x_norm += x(k - 1) * x(k - 1);
    x_diff +=
        (xprev_list[i_c + k] - x(k - 1)) * (xprev_list[i_c + k] - x(k - 1));
    xprev_list[i_c + k] = x(k - 1);
  }
  x_diff = sqrt(x_diff / x_norm);
  double t = estimateMotionPeriod(xprev_list);

  t_diff = fabs(topt - t);
  topt = t;

  TOPT_DEBUG_MSG("xprev_list = ");
  if (TOPT_DEBUG_PRINT) {
    for (int i = 0; i < xprev_list.size(); i++) {
      std::cout << xprev_list[i] << " ";
    }
    std::cout << std::endl;
  }

  xopt_list = xprev_list;
  alpha = 1.0;
  TOPT_DEBUG_MSG("x_diff = " << x_diff << ", t=" << t << ", topt=" << topt 
                 << std::endl);

  int iter = 0;
  while (++iter < 10 && x_diff > 1e-2 && t_diff > 0.01) {
    // build A, b : x[i_c+1:i_c+nh]
    updateTOPP3Ineq(xprev_list, i_c, nh, constraints, alpha);
    int n = constraints[2].b.size();
    A.bottomRows(n) = constraints[2].A;
    b.tail(n) = constraints[2].b;
    f = getCostCoeffs(xprev_list, i_c, nh);
    double ret = lpsolver_->solve(f, A, b, x);

    // update xnew
    x_norm = 0.;
    x_diff = 0.;
    for (int k(1); k < nh + 1; k++) {
      x_norm += x(k - 1) * x(k - 1);
      x_diff +=
          (xprev_list[i_c + k] - x(k - 1)) * (xprev_list[i_c + k] - x(k - 1));
      xprev_list[i_c + k] = x(k - 1);
    }
    x_diff = sqrt(x_diff / x_norm);
    t = estimateMotionPeriod(xprev_list);
    if (t < topt) {
      alpha *= 1.5;
      t_diff = fabs(topt - t);
      topt = t;
      xopt_list = xprev_list;
    } else {
      alpha *= 0.5;
      xprev_list = xopt_list;
    }
    TOPT_DEBUG_MSG("x_diff = " << x_diff << ", t=" << t << ", topt=" << topt
                               << ", alpha=" << alpha << std::endl);
  }
  trackable_xmax_ = xopt_list;
}

Eigen::VectorXd ToptSolver::getCostCoeffs(const std::vector<double>& x0_list,
                                          int k, int h) {
  // min f'x, with opt vars : x=x[k+1]~x[k+h]
  Eigen::VectorXd f = Eigen::VectorXd::Constant(h, -1);

  double xk0, xk1, xk2, dsk0, dsk1;
  for (int i(0); i < h; ++i) {
    xk0 = sqrt(std::max(0.0, x0_list[k + i]));
    xk1 = sqrt(std::max(MIN_SDOT_, x0_list[k + i + 1]));
    xk2 = sqrt(std::max(0.0, x0_list[k + i + 2]));

    dsk0 = sysdata_.s[k + i + 2] - sysdata_.s[k + i + 1];
    dsk1 = sysdata_.s[k + i + 1] - sysdata_.s[k + i];

    f[i] =
        -(dsk0 / (xk0 + xk1) / (xk0 + xk1) + dsk1 / (xk2 + xk1) / (xk2 + xk1)) /
        xk1;
  }
  return f;
}

void ToptSolver::updateTOPP3Ineq(const std::vector<double>& x0_list, int k,
                                 int h, InequalDataList& TOPP3Ineq,
                                 double alpha) {
  // update 3rd order constraints for x[k+1]~x[k+h]
  InequalData cntrt_tmp;

  // Assume x[k](& x[k-1]) are fixed
  // x[k+h+1] is fixed if it's the last point
  bool b_last_fixed = (k + h + 1 == x0_list.size() - 1);

  int i0 = 0;
  int dim3rdIneq = dimIneq3_ * (h - 1);
  if (k > 0) {
    dim3rdIneq += dimIneq3_;
    i0 = 1;
  }
  if (b_last_fixed) dim3rdIneq += dimIneq3_;

  Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero(dim3rdIneq, h);
  Eigen::VectorXd b3 = Eigen::VectorXd::Zero(dim3rdIneq);

  double xk = x0_list[k];

  if (k > 0) {
    // x[k-1], x[k] fixed
    // A2*x[k+1] < b - A0*x[k-1] - A1*x[k]
    double xkpre = x0_list[k - 1];
    get3rdlimit(k - 1, x0_list, cntrt_tmp);
    A3.block(0, 0, dimIneq3_, 1) = cntrt_tmp.A.rightCols(1);
    b3.segment(0, dimIneq3_) =
        cntrt_tmp.b - cntrt_tmp.A.col(1) * xk - cntrt_tmp.A.col(0) * xkpre;
  }
  // A1*x[k+1] +A2*x[k+2] < b - A0*x[k]
  get3rdlimit(k, x0_list, cntrt_tmp);
  A3.block(i0 * dimIneq3_, 0, dimIneq3_, 2) = cntrt_tmp.A.rightCols(2);
  b3.segment(i0 * dimIneq3_, dimIneq3_) = cntrt_tmp.b - cntrt_tmp.A.col(0) * xk;
  for (int i(1); i < h - 1; i++) {
    // A0*x[k+i] + A1*x[k+i+1] +A2*x[k+i+2] < b
    get3rdlimit(k + i, x0_list, cntrt_tmp);
    A3.block((i0 + i) * dimIneq3_, i - 1, dimIneq3_, 3) = cntrt_tmp.A;
    b3.segment((i0 + i) * dimIneq3_, dimIneq3_) = cntrt_tmp.b;
  }
  if (b_last_fixed) {
    // k+h+1 = N-1 : last element
    // A0*x[k+h-1] + A1*x[k+h] < b - A2*{x[k+h+1]=0}
    get3rdlimit(k + h - 1, x0_list, cntrt_tmp);
    A3.bottomRightCorner(dimIneq3_, 2) = cntrt_tmp.A.leftCols(2);
    b3.tail(dimIneq3_) = cntrt_tmp.b;
  }

  // end condition
  if (k == 0) {
    // add jerk constraint with zero acc assumption at the begining
    // qddot[0] / 0.5*dt[0] < jm
    // (dq[0]/2/ds/ds)*x[1]^3/2 < Jm
    double ds = sysdata_.s[1] - sysdata_.s[0];
    Eigen::VectorXd dq0 = sysdata_.dq[0] / 2. / ds / ds;
    double tmp = rossy_utils::getMaxRatioValue(dq0, sysdata_.jm[0]);
    // x[1] < (1/tmp)^2/3
    tmp = std::pow(1. / tmp, 2. / 3.);
    Eigen::MatrixXd Atmp = Eigen::MatrixXd::Zero(1, h);
    Atmp(0, 0) = 1.;
    A3 = rossy_utils::vStack(A3, Atmp);
    b3 = rossy_utils::vStack(b3, tmp);
  }
  if (b_last_fixed) {
    // k+h+1 = N-1 : last element
    // qddot[N-2] / 0.5*dt[N-2] < jm
    // (ddq[N-2]-dq[N-2]/2/ds)/ds * x[N-2]^3/2 < jm
    double ds = sysdata_.s[k + h + 1] - sysdata_.s[k + h];
    Eigen::VectorXd dq0 =
        (sysdata_.ddq[k + h] - sysdata_.dq[k + h] / 2. / ds) / ds;
    double tmp = rossy_utils::getMaxRatioValue(dq0, sysdata_.jm[k + h + 1]);
    // x[N-2] < (1/tmp)^2/3
    tmp = std::pow(1. / tmp, 2. / 3.);
    Eigen::MatrixXd Atmp = Eigen::MatrixXd::Zero(1, h);
    Atmp(0, h - 1) = 1.;
    A3 = rossy_utils::vStack(A3, Atmp);
    b3 = rossy_utils::vStack(b3, tmp);
  }
  //
  if (alpha > 0.) {
    // x - r < x[k+1]~x[k+h] < x + r
    Eigen::MatrixXd Atmp = Eigen::MatrixXd::Identity(h, h);
    Eigen::VectorXd btmp = Eigen::VectorXd::Zero(2 * h);
    for (int i(0); i < h; i++) {
      double r = alpha * x0_list[k + 1 + i];
      btmp(i) = x0_list[k + 1 + i] + r;
      btmp(h + i) = -x0_list[k + 1 + i] + r;
    }
    Atmp = rossy_utils::vStack(Atmp, -Atmp);
    A3 = rossy_utils::vStack(A3, Atmp);
    b3 = rossy_utils::vStack(b3, btmp);
  }

  TOPP3Ineq[2].A = A3;
  TOPP3Ineq[2].b = b3;
}

InequalDataList ToptSolver::buildTOPP3Ineq(const std::vector<double>& x0_list,
                                           int k, int h, double alpha) {
  // Build constraitns for x[k+1:k+h]
  InequalDataList result;
  InequalData cntrt_tmp;

  // Assume x[k](& x[k-1]) are fixed
  // x[k+h+1] is fixed if it's the last point
  bool b_last_fixed = (k + h + 1 == x0_list.size() - 1);

  // 1. 1st order constraints - vel limits
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(dimIneq1_ * h, h);
  Eigen::VectorXd b1 = Eigen::VectorXd::Zero(dimIneq1_ * h);
  for (int i(0); i < h; ++i) {
    get1stlimit(k + 1 + i, cntrt_tmp);
    A1.block(i * dimIneq1_, i, dimIneq1_, 1) = cntrt_tmp.A.col(0);
    b1.segment(i * dimIneq1_, dimIneq1_) = cntrt_tmp.b;
  }
  result[0].A = A1;
  result[0].b = b1;

  // 2. 2nd order constraints - trq/acc limits
  int dim2ndIneq = dimIneq2_ * h;
  if (b_last_fixed) dim2ndIneq += dimIneq2_;
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(dim2ndIneq, h);
  Eigen::VectorXd b2 = Eigen::VectorXd::Zero(dim2ndIneq);

  // A1*x[k+1] < b - A0*x[k]
  double xk = x0_list[k];
  get2ndlimit(k, cntrt_tmp);
  A2.block(0, 0, dimIneq2_, 1) = cntrt_tmp.A.rightCols(1);
  b2.segment(0, dimIneq2_) = cntrt_tmp.b - cntrt_tmp.A.col(0) * xk;
  for (int i(1); i < h; i++) {
    // A0*x[k+i] + A1*x[k+i+1] < b
    get2ndlimit(k + i, cntrt_tmp);
    A2.block(i * dimIneq2_, i - 1, dimIneq2_, 2) = cntrt_tmp.A;
    b2.segment(i * dimIneq2_, dimIneq2_) = cntrt_tmp.b;
  }
  if (b_last_fixed) {
    // A0*x[k+h] < b - A1*x[k+h+1]
    get2ndlimit(k + h, cntrt_tmp);
    A2.block(h * dimIneq2_, h - 1, dimIneq2_, 1) = cntrt_tmp.A.leftCols(1);
    b2.segment(h * dimIneq2_, dimIneq2_) = cntrt_tmp.b;
  }
  result[1].A = A2;
  result[1].b = b2;

  // 3. 3rd order - jerk
  updateTOPP3Ineq(x0_list, k, h, result, alpha);  // updating result[2]

  return result;
}

// check functions

Eigen::VectorXd ToptSolver::computeJerk(int k,
                                        const std::vector<double>& x0_list) {
  // k: 0 ~ nwpts-2
  double dsk = sysdata_.s[k + 1] - sysdata_.s[k];
  double dskk = sysdata_.s[k + 2] - sysdata_.s[k + 1];
  double vk0, vk1, vk2;
  vk0 = sqrt(std::max(MIN_SDOT_, x0_list[k]));
  vk1 = sqrt(std::max(MIN_SDOT_, x0_list[k + 1]));
  vk2 = sqrt(std::max(MIN_SDOT_, x0_list[k + 2]));
  double t = dskk / (vk2 + vk1) + dsk / (vk1 + vk0);
  double t2 = 2. * dsk / (vk1 + vk0);

  Eigen::VectorXd J0, J1, J2;
  J0 = sysdata_.dq[k] / 2. / dsk - sysdata_.ddq[k];
  J1 = -sysdata_.dq[k + 1] / 2. / dskk - sysdata_.dq[k] / 2. / dsk +
       sysdata_.ddq[k + 1];
  J2 = sysdata_.dq[k + 1] / 2. / dskk;

  return (J0 * x0_list[k] + J1 * x0_list[k + 1] + J2 * x0_list[k + 2]) / t;
}

double ToptSolver::checkLimits(const std::vector<double>& x0_list) {
  double motion_in_limit = 0.0;
  int num_wpts = sysdata_.q.size();
  int dim_robot = sysdata_.q[0].size();

  Eigen::VectorXd qdotk2, qddotk, qddotk_pre, qdddotk, trqk;
  double dsk, uk, dtk;

  // smax, t, vel, acc, torque, jerk, grasping
  Eigen::MatrixXd check_limits = Eigen::MatrixXd::Zero(num_wpts, 7);
  double temp, duration(0.);
  Eigen::VectorXd fsuc_i = Eigen::VectorXd::Zero(42);
  double grineq(0.);

  for (int k(0); k < num_wpts; ++k) {
    // 1st - velocity : qdot = q'*sdot
    qdotk2 = sysdata_.dq[k].cwiseProduct(sysdata_.dq[k]) * x0_list[k];
    rossy_utils::saveVector(qdotk2, "topt/qdotk2");

    // 2nd
    if (k == num_wpts - 1) {
      dsk = sysdata_.s[k] - sysdata_.s[k - 1];
      uk = 0.;
      dtk = 2. * dsk / (sqrt(x0_list[k]) + sqrt(x0_list[k - 1]));
    } else {
      dsk = sysdata_.s[k + 1] - sysdata_.s[k];
      uk = (x0_list[k + 1] - x0_list[k]) / (2 * dsk);
      dtk = 2. * dsk / (sqrt(x0_list[k + 1]) + sqrt(x0_list[k]));
    }

    // 2nd - acceleration
    qddotk = sysdata_.ddq[k] * (x0_list[k]) + sysdata_.dq[k] * uk;
    rossy_utils::saveVector(qddotk, "topt/qacc");

    // 3rd - jerk
    if (k == 0)
      qdddotk = 2. * qddotk / dtk;
    else if (k == num_wpts - 1)
      qdddotk = -2. * qddotk_pre / dtk;
    else
      qdddotk = computeJerk(k - 1, x0_list);
    rossy_utils::saveVector(qdddotk, "topt/jerk");
    // if(k==0) qddotk_pre = qddotk;
    // qdddotk = (qddotk - qddotk_pre)/dtk;
    qddotk_pre = qddotk;

    if (k < num_wpts - 1) duration += dtk;
  }

  rossy_utils::pretty_print(check_limits, std::cout,
                            "_sdot_dt_vel_acc_trq_jerk_grasp");
  TOPT_DEBUG_MSG("duration = " << duration << std::endl);

  return motion_in_limit;
}

double ToptSolver::estimateMotionPeriod(const std::vector<double>& x0_list) {
  double t(0.);
  double xk0, xk1, dsk;
  for (int i(0); i < x0_list.size() - 1; ++i) {
    xk0 = sqrt(std::max(0.0, x0_list[i]));
    xk1 = sqrt(std::max(0.0, x0_list[i + 1]));
    dsk = sysdata_.s[i + 1] - sysdata_.s[i];
    t += 2. * dsk / (xk0 + xk1);
  }
  return t;
}
