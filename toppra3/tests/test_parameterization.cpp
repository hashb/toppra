#include "bindings/toppra3.hpp"
#include <iostream>
#include <vector>

int main() {
    // Create a simple linear path in 2D
    const int num_dof = 2;  // 2D trajectory
    std::vector<Eigen::VectorXd> waypoints;
    
    // Create start point (0,0)
    Eigen::VectorXd start(num_dof);
    start << 0.0, 0.0;
    waypoints.push_back(start);

    // Create mid1 point (0.3,0.5)
    Eigen::VectorXd mid1(num_dof);
    mid1 << 0.3, 0.5;
    waypoints.push_back(mid1);

    // Create mid2 point (0.7,0.5)
    Eigen::VectorXd mid2(num_dof);
    mid2 << 0.7, 0.5;
    waypoints.push_back(mid2);

    // Create end point (1,1)
    Eigen::VectorXd end(num_dof);
    end << 1.0, 1.0;
    waypoints.push_back(end);
    
    // Create robot limits
    toppra3::RobotLimits limits(num_dof);
    limits.max_joint_velocity << 1.0, 1.0;        // 1 unit/s for each joint
    limits.max_joint_acceleration << 2.0, 2.0;    // 2 units/s^2 for each joint
    limits.max_joint_jerk << 5.0, 5.0;           // 5 units/s^3 for each joint
    
    // Create parameterization solver
    toppra3::Toppra3Parameterization parameterizer(num_dof);
    
    // Solve for time-optimal trajectory
    bool success = parameterizer.solve(waypoints, limits, true);
    
    if (!success) {
        std::cerr << "Failed to find solution!" << std::endl;
        return 1;
    }
    
    std::cout << "Successfully found trajectory solution" << std::endl;
    
    
    return 0;
} 