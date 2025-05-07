#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "toppra/clock.hpp"
#include "toppra/math/linear_interpolator.hpp"
#include "toppra/robot_model.hpp"
#include "toppra/topt_solver.hpp"
#include "toppra/trajectory_manager.hpp"
#include "toppra/user_command.hpp"
#include "toppra/util.hpp"

namespace toppra3 {
void vectorToEigen(const std::vector<double>& vec, Eigen::VectorXd& eigen_vec) {
  eigen_vec = Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

void eigenToVector(const Eigen::VectorXd& eigen_vec, std::vector<double>& vec) {
  vec.resize(eigen_vec.size());
  for (int i = 0; i < eigen_vec.size(); i++) {
    vec[i] = eigen_vec[i];
  }
}

void eigenIsometry3dToVector(const Eigen::Isometry3d& eigen_iso,
                             std::vector<double>& vec) {
  vec.resize(7);
  Eigen::Vector3d translation(eigen_iso.translation());
  Eigen::Quaterniond rotation(eigen_iso.rotation());
  vec[0] = translation[0];
  vec[1] = translation[1];
  vec[2] = translation[2];

  vec[3] = rotation.w();
  vec[4] = rotation.x();
  vec[5] = rotation.y();
  vec[6] = rotation.z();
}

/**
 * @brief Class containing robot joint limits and constraints
 */
class InputData {
 public:
  InputData(int num_joints, std::vector<double> global_max_joint_velocity,
            std::vector<double> global_max_joint_acceleration,
            std::vector<double> global_max_joint_jerk,
            std::vector<double> global_max_joint_torque,
            std::vector<int> segment_indices,
            std::vector<std::vector<double>> waypoint_scale_factors,
            std::vector<double> waypoint_max_cart_vel_mm_per_s,
            std::vector<double> waypoint_max_cart_acc_mm_per_s2,
            std::string frame_name, std::vector<std::vector<double>> waypoints)
      : num_joints_(num_joints),
        global_max_joint_velocity(global_max_joint_velocity),
        global_max_joint_acceleration(global_max_joint_acceleration),
        global_max_joint_jerk(global_max_joint_jerk),
        global_max_joint_torque(global_max_joint_torque),
        segment_indices(segment_indices),
        waypoint_scale_factors(waypoint_scale_factors),
        waypoint_max_cart_vel_mm_per_s(waypoint_max_cart_vel_mm_per_s),
        waypoint_max_cart_acc_mm_per_s2(waypoint_max_cart_acc_mm_per_s2),
        frame_name(frame_name),
        waypoints(waypoints) {}

  // dims
  int num_joints_;

  // Joint limits
  std::vector<double> global_max_joint_velocity;
  std::vector<double> global_max_joint_acceleration;
  std::vector<double> global_max_joint_jerk;
  // torque limits
  std::vector<double> global_max_joint_torque;

  // segment indices
  std::vector<int> segment_indices;
  // scale factors
  std::vector<std::vector<double>> waypoint_scale_factors;
  // cart limits
  std::vector<double> waypoint_max_cart_vel_mm_per_s;
  std::vector<double> waypoint_max_cart_acc_mm_per_s2;
  std::string frame_name;

  // waypoints
  std::vector<std::vector<double>> waypoints;

  // Convert to SYSTEM_DATA format used internally
  SYSTEM_DATA toSystemData(std::shared_ptr<TrajectoryManager>& traj_manager,
                           std::shared_ptr<RobotSystem>& robot_model) const {
    TOPT_DEBUG_MSG(".");
    toppra::math::LinearInterpolator spl_velocity_scale_factors(
        traj_manager->s2q_times_, waypoint_scale_factors[0]);
    TOPT_DEBUG_MSG(".");
    toppra::math::LinearInterpolator spl_acceleration_scale_factors(
        traj_manager->s2q_times_, waypoint_scale_factors[1]);
    TOPT_DEBUG_MSG(".");
    toppra::math::LinearInterpolator spl_jerk_scale_factors(
        traj_manager->s2q_times_, waypoint_scale_factors[2]);
    TOPT_DEBUG_MSG(".");
    toppra::math::LinearInterpolator spl_cart_vel_scale_factors(
        traj_manager->s2q_times_, waypoint_max_cart_vel_mm_per_s);
    TOPT_DEBUG_MSG(".");
    toppra::math::LinearInterpolator spl_cart_acc_scale_factors(
        traj_manager->s2q_times_, waypoint_max_cart_acc_mm_per_s2);

    Eigen::VectorXd global_max_joint_velocity_eigen =
        Eigen::Map<const Eigen::VectorXd>(global_max_joint_velocity.data(),
                                          global_max_joint_velocity.size());
    Eigen::VectorXd global_max_joint_acceleration_eigen =
        Eigen::Map<const Eigen::VectorXd>(global_max_joint_acceleration.data(),
                                          global_max_joint_acceleration.size());
    Eigen::VectorXd global_max_joint_jerk_eigen =
        Eigen::Map<const Eigen::VectorXd>(global_max_joint_jerk.data(),
                                          global_max_joint_jerk.size());

    SYSTEM_DATA sysdata;
    int n = traj_manager->spline_s2q_.getNumWpts();
    sysdata.resize(n);

    // store sys data along the path
    Eigen::VectorXd q, dq, ddq;
    Eigen::MatrixXd J;
    Eigen::VectorXd dJdq;
    Eigen::VectorXd grav{{0., 0., -9.8}};
    Eigen::VectorXd torque_limits;
    vectorToEigen(global_max_joint_torque, torque_limits);

    double s = 0.;
    double ds = 1. / ((double)(n - 1));
    for (int i(0); i < n; ++i) {
      sysdata.s[i] = s;
      s += ds;
    }
    TOPT_DEBUG_MSG(".");
    // Set path waypoints
    for (int i = 0; i < n; i++) {
      s = sysdata.s[i];
      q = traj_manager->spline_s2q_.evaluate(s);
      dq = traj_manager->spline_s2q_.evaluateFirstDerivative(s);
      ddq = traj_manager->spline_s2q_.evaluateSecondDerivative(s);
      robot_model->updateSystem(q, dq);
      sysdata.q[i] = q;
      sysdata.dq[i] = dq;
      sysdata.ddq[i] = ddq;

      // for the cartesian vel/acc constraint
      sysdata.ee[i] =
          robot_model->getBodyNodeIsometry(frame_name).translation();
      J = robot_model->getBodyNodeJacobian(frame_name);
      // J'(q,q')q'
      dJdq = robot_model->getBodyNodeJacobianDotQDot(frame_name);
      sysdata.ee_v[i] = J.bottomRows(3) * dq;
      sysdata.ee_a[i] = J.bottomRows(3) * ddq + dJdq.bottomRows(3);

      // M(q(s))*(q''ds2+q'dds)+C(q, q')q'ds2+g(q(s))
      //  = m dds + b ds2 + g
      // m=Mq', b=M(q)*q''+C(q,q')q', g=g(q)
      sysdata.m[i] = robot_model->getMassMatrix() * dq;
      sysdata.b[i] = robot_model->getMassMatrix() * ddq +
                     robot_model->getCoriolisMatrix() * dq;
      sysdata.g[i] = robot_model->getGravity();
      sysdata.tm[i] = torque_limits;

      // Set limits at each waypoint
      sysdata.av[i] = sysdata.dq[i].cwiseProduct(sysdata.dq[i]);
      Eigen::VectorXd v = global_max_joint_velocity_eigen *
                          spl_velocity_scale_factors.interpolate(s);
      sysdata.vm2[i] = v.cwiseProduct(v);

      sysdata.am[i] = global_max_joint_acceleration_eigen *
                      spl_acceleration_scale_factors.interpolate(s);
      sysdata.jm[i] =
          global_max_joint_jerk_eigen * spl_jerk_scale_factors.interpolate(s);

      // linear vel/acc limits, -1 will be given if inactivated
      sysdata.lvm[i] = spl_cart_vel_scale_factors.interpolate(s) * 1e-3;
      sysdata.lam[i] = spl_cart_acc_scale_factors.interpolate(s) * 1e-3;
    }
    TOPT_DEBUG_MSG("|" << std::endl);
    return sysdata;
  }
};

class TimedWaypoint {
 public:
  TimedWaypoint() = default;

  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;

  std::string frame_name;
  std::vector<double> cart_pos;
  double max_cart_vel;
  double max_cart_acc;

  double time_from_start;
  int segment_index;
  bool is_path_position;
  bool time_from_previous;
};

/**
 * @brief Class containing the output of the time-optimal parameterization
 */
class OutputData {
 public:
  OutputData() = default;
  bool success;
  std::vector<TimedWaypoint> waypoints;
};

/**
 * @brief Main class for time-optimal trajectory parameterization with jerk
 * constraints
 */
class Toppra3Parameterization {
 public:
  Toppra3Parameterization(int num_joints, std::string mjcf_path)
      : num_joints_(num_joints), mjcf_path_(mjcf_path) {
    robot_model_ = std::make_shared<RobotSystem>(mjcf_path);
  }

  /**
   * @brief Solve for time-optimal trajectory
   *
   * @param input_data Robot joint limits and constraints
   * @param use_jerk_limits Whether to include jerk constraints
   * @return true if solution found
   */
  OutputData solve(const InputData& input_data, bool use_jerk_limits = true) {
    Clock clock;
    clock.start();
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve SETUP 0\n");
    std::shared_ptr<TrajectoryManager> traj_manager_ =
        std::make_shared<TrajectoryManager>();
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve SETUP 1 (" << clock.stop()
                                                              << "ms)\n");
    clock.start();
    std::shared_ptr<ToptSolver> solver_ =
        std::make_shared<ToptSolver>(num_joints_);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve SETUP 2 (" << clock.stop()
                                                              << "ms)\n");
    clock.start();

    // Convert waypoints to normalized path
    std::vector<Eigen::VectorXd> normalized_waypoints;
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve PREPROCESSING 3 ("
                   << clock.stop() << "ms)\n");
    clock.start();

    // convert waypoints to eigen vectors
    std::vector<Eigen::VectorXd> eigen_waypoints;
    for (const auto& waypoint : input_data.waypoints) {
      Eigen::VectorXd eigen_waypoint =
          Eigen::Map<const Eigen::VectorXd>(waypoint.data(), waypoint.size());
      eigen_waypoints.push_back(eigen_waypoint);
    }

    traj_manager_->redistQwptsPureNormDist(eigen_waypoints,
                                           normalized_waypoints);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve PREPROCESSING 4 ("
                   << clock.stop() << "ms)\n");
    clock.start();
    traj_manager_->setS2QSpline(normalized_waypoints);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve PREPROCESSING 5 ("
                   << clock.stop() << "ms)\n");
    clock.start();

    // Convert limits to system data format
    SYSTEM_DATA sysdata = input_data.toSystemData(traj_manager_, robot_model_);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve SOLVE 6 (" << clock.stop()
                                                              << "ms)\n");
    clock.start();
    TOPT_DEBUG_MSG("\n");

    // Solve time-optimal parameterization
    bool success =
        solver_->solve(sysdata, traj_manager_.get(), use_jerk_limits);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve POSTPROCESSING 7 ("
                   << clock.stop() << "ms)\n");
    if (!success) {
      OutputData output_data;
      output_data.success = false;
      return output_data;
    }

    // mapping to compute segment index
    // linearly interpolate input_times and segment_indices
    // evaluate spline at gridpoints before parameterization
    // linearly interpolate gridpoints after parameterization and
    // segment_indices at gridpoints before parameterization
    std::vector<double> segment_indicies_as_double;
    for (const auto& segment_index : input_data.segment_indices) {
      segment_indicies_as_double.push_back(static_cast<double>(segment_index));
    }
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve POSTPROCESSING 8 ("
                   << clock.stop() << "ms)\n");
    clock.start();
    toppra::math::LinearInterpolator spl_segment_indicies_at_input_times(
        traj_manager_->s2q_times_, segment_indicies_as_double);
    std::vector<double> segment_indices_at_gridpoints;
    segment_indices_at_gridpoints.resize(traj_manager_->s2q_gridpoints_.size());
    for (int i = 0; i < traj_manager_->s2q_gridpoints_.size(); i++) {
      segment_indices_at_gridpoints[i] =
          spl_segment_indicies_at_input_times.interpolate(
              traj_manager_->s2q_gridpoints_[i]);
    }
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve POSTPROCESSING 9 ("
                   << clock.stop() << "ms)\n");
    clock.start();
    toppra::math::LinearInterpolator
        spl_segment_indicies_at_parameterized_times(
            traj_manager_->t2q_gridpoints_, segment_indices_at_gridpoints);
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve POSTPROCESSING 10 ("
                   << clock.stop() << "ms)\n");
    clock.start();

    // Interpolate times
    std::vector<double> interpolated_times;
    double duration = traj_manager_->getMotionPeriod();
    double interval = 0.01;  // 10ms

    int num_points = std::ceil(duration / interval) + 1;
    interpolated_times.resize(num_points);
    for (int i = 0; i < num_points; i++) {
      interpolated_times[i] = i * (duration / (num_points - 1));
    }

    // copy data to output data
    OutputData output_data;
    Eigen::VectorXd q_cmd;
    Eigen::VectorXd qdot_cmd;
    Eigen::VectorXd qddot_cmd;
    Eigen::MatrixXd J;
    Eigen::VectorXd dJdq;

    for (int i = 0; i < num_points; i++) {
      traj_manager_->getCommand(interpolated_times[i], q_cmd, qdot_cmd,
                                qddot_cmd);
      TimedWaypoint timed_waypoint;
      eigenToVector(q_cmd, timed_waypoint.q);
      eigenToVector(qdot_cmd, timed_waypoint.dq);
      eigenToVector(qddot_cmd, timed_waypoint.ddq);
      timed_waypoint.time_from_start = interpolated_times[i];
      if (i == 0) {
        timed_waypoint.time_from_previous = 0.0;
      } else {
        timed_waypoint.time_from_previous =
            interpolated_times[i] - interpolated_times[i - 1];
      }
      timed_waypoint.segment_index =
          spl_segment_indicies_at_parameterized_times.interpolate(
              interpolated_times[i]);
      timed_waypoint.is_path_position = true;

      // get ee pos, vel, acc
      robot_model_->updateSystem(q_cmd, qdot_cmd);
      timed_waypoint.frame_name = input_data.frame_name;
      eigenIsometry3dToVector(
          robot_model_->getBodyNodeIsometry(input_data.frame_name),
          timed_waypoint.cart_pos);

      // Jacobian and its derivative
      J = robot_model_->getBodyNodeJacobian(input_data.frame_name);
      dJdq = robot_model_->getBodyNodeJacobianDotQDot(input_data.frame_name);

      // Compute linear Cartesian velocity and acceleration (first 3 components)
      // and keep only their maximum value.
      timed_waypoint.max_cart_vel = (J.bottomRows(3) * qdot_cmd).maxCoeff();

      timed_waypoint.max_cart_acc =
          (J.bottomRows(3) * qddot_cmd + dJdq.bottomRows(3)).maxCoeff();

      output_data.waypoints.push_back(timed_waypoint);
    }
    TOPT_DEBUG_MSG("Toppra3Parameterization::solve POSTPROCESSING 11 ("
                   << clock.stop() << "ms)\n");

    output_data.success = success;
    return output_data;
  }

  int getNumJoints() const { return num_joints_; }

 private:
  int num_joints_;
  std::string mjcf_path_;
  std::shared_ptr<RobotSystem> robot_model_;
};

}  // namespace toppra3
