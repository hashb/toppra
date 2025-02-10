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
    
    // Get trajectory duration
    double duration = parameterizer.getDuration();
    std::cout << "Trajectory duration: " << duration << " seconds" << std::endl;
    
    // Test some points along the trajectory
    std::vector<double> test_times = {0.0, duration/2.0, duration};
    
    for (double t : test_times) {
        Eigen::VectorXd position(num_dof);
        Eigen::VectorXd velocity(num_dof);
        Eigen::VectorXd acceleration(num_dof);
        
        parameterizer.getState(t, position, velocity, acceleration);
        
        std::cout << "\nAt time " << t << ":" << std::endl;
        std::cout << "Position: " << position.transpose() << std::endl;
        std::cout << "Velocity: " << velocity.transpose() << std::endl;
        std::cout << "Acceleration: " << acceleration.transpose() << std::endl;
        
        // Basic checks
        if (velocity.norm() > limits.max_joint_velocity.norm()) {
            std::cerr << "Warning: Velocity limit exceeded!" << std::endl;
        }
        if (acceleration.norm() > limits.max_joint_acceleration.norm()) {
            std::cerr << "Warning: Acceleration limit exceeded!" << std::endl;
        }
    }
    
    return 0;
} 