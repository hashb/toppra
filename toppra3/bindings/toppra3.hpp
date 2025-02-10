#pragma once

#include <Eigen/Dense>
#include <vector>
#include "toppra/user_command.hpp"
#include "toppra/topt_solver.hpp"
#include "toppra/trajectory_manager.hpp"

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
        
        max_linear_velocity = -1;
        max_linear_acceleration = -1;
    }

    // Joint limits
    Eigen::VectorXd max_joint_velocity;
    Eigen::VectorXd max_joint_acceleration; 
    Eigen::VectorXd max_joint_jerk;
    Eigen::VectorXd max_joint_torque;

    // End-effector limits
    double max_linear_velocity;
    double max_linear_acceleration;

    // Convert to SYSTEM_DATA format used internally
    SYSTEM_DATA toSystemData(const std::vector<Eigen::VectorXd>& waypoints) const {
        SYSTEM_DATA sysdata;
        int n = waypoints.size();
        sysdata.resize(n);

        // Set path waypoints
        for(int i = 0; i < n; i++) {
            sysdata.s[i] = i / double(n-1); // Normalize path parameter to [0,1]
            sysdata.q[i] = waypoints[i];
            
            // Set limits at each waypoint
            sysdata.vm2[i] = max_joint_velocity.cwiseProduct(max_joint_velocity);
            sysdata.am[i] = max_joint_acceleration;
            sysdata.jm[i] = max_joint_jerk;
            sysdata.tm[i] = max_joint_torque;
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
        num_joints_(num_joints),
        solver_(num_joints),
        traj_manager_() {}

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
        
        // Convert waypoints to normalized path
        std::vector<Eigen::VectorXd> normalized_waypoints;
        traj_manager_.redistQwptsPureNormDist(waypoints, normalized_waypoints);

        // Convert limits to system data format
        SYSTEM_DATA sysdata = limits.toSystemData(normalized_waypoints);

        // Solve time-optimal parameterization
        bool success = solver_.solve(sysdata, &traj_manager_, use_jerk_limits);

        return success;
    }

    /**
     * @brief Get trajectory state at time t
     * 
     * @param time Time along trajectory
     * @param position Output joint positions
     * @param velocity Output joint velocities 
     * @param acceleration Output joint accelerations
     */
    void getState(double time,
                 Eigen::VectorXd& position,
                 Eigen::VectorXd& velocity,
                 Eigen::VectorXd& acceleration) {
        traj_manager_.getCommand(time, position, velocity, acceleration);
    }

    /**
     * @brief Get total trajectory duration
     */
    double getDuration() {
        std::vector<double> periods;
        traj_manager_.getMotionPeriods(periods);
        return periods.back();
    }

    // Add this getter method
    int getNumJoints() const { return num_joints_; }

private:
    int num_joints_;
    ToptSolver solver_;
    TrajectoryManager traj_manager_;
};
