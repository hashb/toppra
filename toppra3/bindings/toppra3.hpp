#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "rossy_utils/general/clock.hpp"
#include "rossy_utils/math/linear_interpolator.hpp"
#include "toppra/topt_solver.hpp"
#include "toppra/trajectory_manager.hpp"
#include "toppra/user_command.hpp"

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

/**
 * @brief Class containing robot joint limits and constraints
 */
class InputData {
 public:
  InputData(int num_joints, std::vector<double> max_joint_velocity,
            std::vector<double> max_joint_acceleration,
            std::vector<double> max_joint_jerk, std::vector<int> segment_indices,
            std::vector<std::vector<double>> scale_factors,
            std::vector<std::vector<double>> waypoints)
      : num_joints_(num_joints),
        max_joint_velocity(max_joint_velocity),
        max_joint_acceleration(max_joint_acceleration),
        max_joint_jerk(max_joint_jerk),
        segment_indices(segment_indices),
        scale_factors(scale_factors),
        waypoints(waypoints) {}

  // dims
  int num_joints_;

  // Joint limits
  std::vector<double> max_joint_velocity;
  std::vector<double> max_joint_acceleration;
  std::vector<double> max_joint_jerk;

  // segment indices
  std::vector<int> segment_indices;

  // waypoints
  std::vector<std::vector<double>> waypoints;

  // scale factors
  std::vector<std::vector<double>> scale_factors;

  // Convert to SYSTEM_DATA format used internally
  SYSTEM_DATA toSystemData(std::shared_ptr<TrajectoryManager>& traj_manager) const {
    rossy_utils::math::LinearInterpolator spl_velocity_scale_factors(traj_manager->s2q_times_, scale_factors[0]);
    rossy_utils::math::LinearInterpolator spl_acceleration_scale_factors(traj_manager->s2q_times_, scale_factors[1]);
    rossy_utils::math::LinearInterpolator spl_jerk_scale_factors(traj_manager->s2q_times_, scale_factors[2]);

    Eigen::VectorXd max_joint_velocity_eigen = Eigen::Map<const Eigen::VectorXd>(max_joint_velocity.data(), max_joint_velocity.size());
    Eigen::VectorXd max_joint_acceleration_eigen = Eigen::Map<const Eigen::VectorXd>(max_joint_acceleration.data(), max_joint_acceleration.size());
    Eigen::VectorXd max_joint_jerk_eigen = Eigen::Map<const Eigen::VectorXd>(max_joint_jerk.data(), max_joint_jerk.size());

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
      Eigen::VectorXd v = max_joint_velocity_eigen * spl_velocity_scale_factors.interpolate(s);
      sysdata.vm2[i] = v.cwiseProduct(v);

      sysdata.am[i] = max_joint_acceleration_eigen * spl_acceleration_scale_factors.interpolate(s);
      sysdata.jm[i] = max_joint_jerk_eigen * spl_jerk_scale_factors.interpolate(s);
    }

    return sysdata;
  }
};

class TimedWaypoint {
 public:
  TimedWaypoint() = default;
  
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> ddq;

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
 * @brief Main class for time-optimal trajectory parameterization with jerk constraints
 */
class Toppra3Parameterization {
 public:
  Toppra3Parameterization(int num_joints) : num_joints_(num_joints) {}

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
    std::cout << "Toppra3Parameterization::solve SETUP 0" << std::endl;
    std::shared_ptr<TrajectoryManager> traj_manager_ =
        std::make_shared<TrajectoryManager>();
    std::cout << "Toppra3Parameterization::solve SETUP 1 (" << clock.stop() << "ms)"
              << std::endl;
    clock.start();
    std::shared_ptr<ToptSolver> solver_ = std::make_shared<ToptSolver>(num_joints_);
    std::cout << "Toppra3Parameterization::solve SETUP 2 (" << clock.stop() << "ms)"
              << std::endl;
    clock.start();
    
    // Convert waypoints to normalized path
    std::vector<Eigen::VectorXd> normalized_waypoints;
    std::cout << "Toppra3Parameterization::solve PREPROCESSING 3 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();

    // convert waypoints to eigen vectors
    std::vector<Eigen::VectorXd> eigen_waypoints;
    for (const auto& waypoint : input_data.waypoints) {
      Eigen::VectorXd eigen_waypoint = Eigen::Map<const Eigen::VectorXd>(waypoint.data(), waypoint.size());
      eigen_waypoints.push_back(eigen_waypoint);
    }

    traj_manager_->redistQwptsPureNormDist(eigen_waypoints, normalized_waypoints);
    std::cout << "Toppra3Parameterization::solve PREPROCESSING 4 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();
    traj_manager_->setS2QSpline(normalized_waypoints);
    std::cout << "Toppra3Parameterization::solve PREPROCESSING 5 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();
    
    // Convert limits to system data format
    SYSTEM_DATA sysdata = input_data.toSystemData(traj_manager_);
    std::cout << "Toppra3Parameterization::solve SOLVE 6 (" << clock.stop() << "ms)"
              << std::endl;
    clock.start();
    std::cout << std::endl;
    
    // Solve time-optimal parameterization
    bool success = solver_->solve(sysdata, traj_manager_.get(), use_jerk_limits);
    std::cout << std::endl;
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 7 (" << clock.stop()
              << "ms)" << std::endl;
    if (!success) {
      OutputData output_data;
      output_data.success = false;
      return output_data;
    }

    // mapping to compute segment index
    // linearly interpolate input_times and segment_indices
    // evaluate spline at gridpoints before parameterization
    // linearly interpolate gridpoints after parameterization and segment_indices at gridpoints before parameterization
    std::vector<double> segment_indicies_as_double;
    for (const auto& segment_index : input_data.segment_indices) {
        segment_indicies_as_double.push_back(static_cast<double>(segment_index));
    }
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 8 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();
    rossy_utils::math::LinearInterpolator spl_segment_indicies_at_input_times(traj_manager_->s2q_times_, segment_indicies_as_double);
    std::vector<double> segment_indices_at_gridpoints;
    segment_indices_at_gridpoints.resize(traj_manager_->s2q_gridpoints_.size());
    for (int i = 0; i < traj_manager_->s2q_gridpoints_.size(); i++) {
        segment_indices_at_gridpoints[i] = spl_segment_indicies_at_input_times.interpolate(traj_manager_->s2q_gridpoints_[i]);
    }
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 9 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();
    rossy_utils::math::LinearInterpolator spl_segment_indicies_at_parameterized_times(traj_manager_->t2q_gridpoints_, segment_indices_at_gridpoints);
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 10 (" << clock.stop()
              << "ms)" << std::endl;
    clock.start();


    // Interpolate times
    std::vector<double> interpolated_times;
    double duration = traj_manager_->getMotionPeriod();
    double interval = 0.01; // 10ms

    int num_points = std::ceil(duration / interval) + 1;
    interpolated_times.resize(num_points);
    for (int i = 0; i < num_points; i++) {
        interpolated_times[i] = i * (duration / (num_points - 1));
    }

    // copy data to output data
    OutputData output_data;

    for (int i = 0; i < num_points; i++) {
        Eigen::VectorXd q_cmd;
        Eigen::VectorXd qdot_cmd;
        Eigen::VectorXd qddot_cmd;
        traj_manager_->getCommand(interpolated_times[i], q_cmd, qdot_cmd, qddot_cmd);
        TimedWaypoint timed_waypoint;
        eigenToVector(q_cmd, timed_waypoint.q);
        eigenToVector(qdot_cmd, timed_waypoint.dq);
        eigenToVector(qddot_cmd, timed_waypoint.ddq);
        timed_waypoint.time_from_start = interpolated_times[i];
        if (i == 0) {
          timed_waypoint.time_from_previous = 0.0;
        } else {
          timed_waypoint.time_from_previous = interpolated_times[i] - interpolated_times[i-1];
        }
        timed_waypoint.segment_index = spl_segment_indicies_at_parameterized_times.interpolate(interpolated_times[i]);
        timed_waypoint.is_path_position = true;
        output_data.waypoints.push_back(timed_waypoint);

    }
    std::cout << "Toppra3Parameterization::solve POSTPROCESSING 11 (" << clock.stop()
              << "ms)" << std::endl;


    output_data.success = success;
    return output_data;
  }

  int getNumJoints() const { return num_joints_; }

 private:
  int num_joints_;
};

}  // namespace toppra3
