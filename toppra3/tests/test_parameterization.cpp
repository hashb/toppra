#include "bindings/toppra3.hpp"
#include <iostream>
#include <vector>

int main() {
    // Create a simple linear path in 2D
    const int num_dof = 2;  // 2D trajectory
    std::vector<std::vector<double>> waypoints;
    
    // Create start point (0,0)
    std::vector<double> start(num_dof);
    start[0] = 0.0;
    start[1] = 0.0;
    waypoints.push_back(start);

    // Create mid1 point (0.3,0.5)
    std::vector<double> mid1(num_dof);
    mid1[0] = 0.3;
    mid1[1] = 0.5;
    waypoints.push_back(mid1);

    // Create mid2 point (0.7,0.5)
    std::vector<double> mid2(num_dof);
    mid2[0] = 0.7;
    mid2[1] = 0.5;
    waypoints.push_back(mid2);

    // Create end point (1,1)
    std::vector<double> end(num_dof);
    end[0] = 1.0;
    end[1] = 1.0;
    waypoints.push_back(end);
    
    // Create robot limits
    std::vector<double> max_joint_velocity(num_dof);
    max_joint_velocity[0] = 1.0;
    max_joint_velocity[1] = 1.0;
    std::vector<double> max_joint_acceleration(num_dof);
    max_joint_acceleration[0] = 2.0;
    max_joint_acceleration[1] = 2.0;
    std::vector<double> max_joint_jerk(num_dof);
    max_joint_jerk[0] = 5.0;
    max_joint_jerk[1] = 5.0;
    std::vector<int> segment_indices(waypoints.size());
    for (int i = 0; i < waypoints.size(); i++) {
        segment_indices[i] = i;
    }
    std::vector<std::vector<double>> scale_factors(3);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < waypoints.size(); j++) {
            scale_factors[i].push_back(1.0);
        }
    }
    toppra3::InputData limits(num_dof, max_joint_velocity, max_joint_acceleration, max_joint_jerk, segment_indices, scale_factors, waypoints);

    // Create parameterization solver
    toppra3::Toppra3Parameterization parameterizer(num_dof);
    
    // Solve for time-optimal trajectory
    toppra3::OutputData result = parameterizer.solve(limits);
    
    if (!result.success) {
        std::cerr << "Failed to find solution!" << std::endl;
        return 1;
    }
    
    std::cout << "Successfully found trajectory solution" << std::endl;
    
    
    return 0;
} 