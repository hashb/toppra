#pragma once

#include <Eigen/Dense>
#include <vector>
#include "toppra/user_command.hpp"
#include "toppra/topt_solver.hpp"
#include "toppra/trajectory_manager.hpp"


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
    SYSTEM_DATA toSystemData(const std::shared_ptr<TrajectoryManager>& traj_manager) const {
        SYSTEM_DATA sysdata;
        int n = traj_manager->spline_s2q_.get_num_segments();
        sysdata.resize(n);

        // Set path waypoints
        for(int i = 0; i < n; i++) {
            sysdata.s[i] = i / double(n-1); // Normalize path parameter to [0,1]
            sysdata.q[i] = traj_manager->spline_s2q_.evaluate(sysdata.s[i]);
            sysdata.dq[i] = traj_manager->spline_s2q_.evaluateFirstDerivative(sysdata.s[i]);
            sysdata.ddq[i] = traj_manager->spline_s2q_.evaluateSecondDerivative(sysdata.s[i]);
            
            // Set limits at each waypoint
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
    Toppra3Parameterization(int num_joints) : 
        num_joints_(num_joints) {}

    /**
     * @brief Solve for time-optimal trajectory
     * 
     * @param waypoints List of joint position waypoints
     * @param limits Robot joint limits and constraints
     * @param use_jerk_limits Whether to include jerk constraints
     * @return true if solution found
     */
    bool solve(const std::vector<Eigen::VectorXd>& waypoints,
              const RobotLimits& limits,
              bool use_jerk_limits = true) {

        std::cout << "Toppra3Parameterization::solve 0" << std::endl;
        std::shared_ptr<TrajectoryManager> traj_manager_ = std::make_shared<TrajectoryManager>();
        std::cout << "Toppra3Parameterization::solve 1" << std::endl;
        std::shared_ptr<ToptSolver> solver_ = std::make_shared<ToptSolver>(num_joints_);
        std::cout << "Toppra3Parameterization::solve 2" << std::endl;
        
        // Convert waypoints to normalized path
        std::vector<Eigen::VectorXd> normalized_waypoints;
        std::cout << "Toppra3Parameterization::solve 3" << std::endl;
        traj_manager_->redistQwptsPureNormDist(waypoints, normalized_waypoints);
        std::cout << "Toppra3Parameterization::solve 4" << std::endl;
        // Convert limits to system data format
        SYSTEM_DATA sysdata = limits.toSystemData(normalized_waypoints);
        std::cout << "Toppra3Parameterization::solve 5" << std::endl;
        // Solve time-optimal parameterization
        bool success = solver_->solve(sysdata, traj_manager_.get(), use_jerk_limits);
        std::cout << "Toppra3Parameterization::solve 6" << std::endl;
        return success;
    }

    // Add this getter method
    int getNumJoints() const { return num_joints_; }

private:
    int num_joints_;
};

}  // namespace toppra3
