#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "toppra/topt_solver.hpp"
#include "toppra/trajectory_manager.hpp"
#include "toppra/user_command.hpp"

namespace toppra3 {
/**
 * @brief Class containing robot joint limits and constraints
 */
class RobotLimits {
 public:
  RobotLimits(int num_joints) {
    max_joint_velocity = Eigen::VectorXd::Zero(num_joints);
    max_joint_acceleration = Eigen::VectorXd::Zero(num_joints);
    max_joint_jerk = Eigen::VectorXd::Zero(num_joints);
    max_joint_torque = Eigen::VectorXd::Zero(num_joints);
  }

  // Joint limits
  Eigen::VectorXd max_joint_velocity;
  Eigen::VectorXd max_joint_acceleration;
  Eigen::VectorXd max_joint_jerk;
  Eigen::VectorXd max_joint_torque;

  // Convert to SYSTEM_DATA format used internally
  SYSTEM_DATA toSystemData(std::shared_ptr<TrajectoryManager>& traj_manager) const {
    SYSTEM_DATA sysdata;
    int n = traj_manager->spline_s2q_.getNumWpts();
    sysdata.resize(n);

    double s = 0.;
    double ds = 1. / ((double)(n - 1));
    for (int i(0); i < n; ++i) {
      sysdata.s[i] = s;
      s += ds;
    }

    // Set path waypoints
    for (int i = 0; i < n; i++) {
      s = sysdata.s[i];
      sysdata.q[i] = traj_manager->spline_s2q_.evaluate(s);
      sysdata.dq[i] = traj_manager->spline_s2q_.evaluateFirstDerivative(s);
      sysdata.ddq[i] = traj_manager->spline_s2q_.evaluateSecondDerivative(s);

      // Set limits at each waypoint
      sysdata.av[i] = sysdata.dq[i].cwiseProduct(sysdata.dq[i]);
      sysdata.vm2[i] = max_joint_velocity.cwiseProduct(max_joint_velocity);

      sysdata.am[i] = max_joint_acceleration;
      sysdata.jm[i] = max_joint_jerk;
    }

    return sysdata;
  }
};

/**
 * @brief Main class for time-optimal trajectory parameterization with jerk constraints
 */
class Toppra3Parameterization {
 public:
  Toppra3Parameterization(int num_joints) : num_joints_(num_joints) {}

  /**
   * @brief Solve for time-optimal trajectory
   *
   * @param waypoints List of joint position waypoints
   * @param limits Robot joint limits and constraints
   * @param use_jerk_limits Whether to include jerk constraints
   * @return true if solution found
   */
  bool solve(const std::vector<Eigen::VectorXd>& waypoints, const RobotLimits& limits,
             bool use_jerk_limits = true) {
    std::cout << "Toppra3Parameterization::solve SETUP 0" << std::endl;
    std::shared_ptr<TrajectoryManager> traj_manager_ =
        std::make_shared<TrajectoryManager>();
    std::cout << "Toppra3Parameterization::solve SETUP 1" << std::endl;
    std::shared_ptr<ToptSolver> solver_ = std::make_shared<ToptSolver>(num_joints_);
    std::cout << "Toppra3Parameterization::solve SETUP 2" << std::endl;

    // Convert waypoints to normalized path
    std::vector<Eigen::VectorXd> normalized_waypoints;
    std::cout << "Toppra3Parameterization::solve PREPROCESSING 3" << std::endl;
    traj_manager_->redistQwptsPureNormDist(waypoints, normalized_waypoints);
    std::cout << "Toppra3Parameterization::solve PREPROCESSING 4" << std::endl;

    // Convert limits to system data format
    SYSTEM_DATA sysdata = limits.toSystemData(traj_manager_);
    std::cout << "Toppra3Parameterization::solve SOLVE 5" << std::endl;

    // Solve time-optimal parameterization
    bool success = solver_->solve(sysdata, traj_manager_.get(), use_jerk_limits);
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 6" << std::endl;
    return success;
  }

  // Add this getter method
  int getNumJoints() const { return num_joints_; }

 private:
  int num_joints_;
};

}  // namespace toppra3
