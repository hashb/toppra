#include <iostream>
#include <memory>
#include <toppra/toppra.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/joint_torque.hpp>
#include <toppra/constraint/joint_jerk.hpp>
#include <toppra/algorithm/toppra3.hpp>
#include <toppra/parametrizer/spline.hpp>

using namespace toppra;

int main() {
    // Create a simple 2-DOF path
    int dof = 2;
    int N = 5;  // Number of waypoints
    
    // Define waypoints
    Vectors waypoints(N);
    waypoints[0] = Vector::Zero(dof);
    waypoints[1] = (Vector(dof) << 0.5, 0.3).finished();
    waypoints[2] = (Vector(dof) << 1.0, 0.8).finished();
    waypoints[3] = (Vector(dof) << 1.5, 1.2).finished();
    waypoints[4] = (Vector(dof) << 2.0, 1.5).finished();
    
    // Create time intervals (uniform)
    Vector times = Vector::LinSpaced(N, 0, 1);
    
    // Create a piecewise polynomial path
    auto path = std::make_shared<geometric_path::PiecewisePolyPath>(waypoints, times);
    
    // Define constraints
    LinearConstraintPtrs constraints;
    
    // 1. Joint velocity limits
    Vector vel_limits = (Vector(dof) << 2.0, 2.0).finished();
    auto vel_constraint = std::make_shared<constraint::LinearJointVelocity>(vel_limits);
    constraints.push_back(vel_constraint);
    
    // 2. Joint acceleration limits
    Vector acc_limits = (Vector(dof) << 4.0, 4.0).finished();
    auto acc_constraint = std::make_shared<constraint::LinearJointAcceleration>(acc_limits);
    constraints.push_back(acc_constraint);
    
    // 3. Joint jerk limits (new!)
    Vector jerk_limits = (Vector(dof) << 10.0, 10.0).finished();
    auto jerk_constraint = std::make_shared<constraint::JointJerkConstraint>(jerk_limits);
    constraints.push_back(jerk_constraint);
    
    // Create TOPPRA3 solver
    auto algo = std::make_shared<algorithm::TOPPRA3>(constraints, path);
    
    // Set solver parameters
    algo->setN(100);  // Number of grid points
    algo->setTOPP3Parameters(10, 0.5, 1e-6);  // max_iter, trust_region, tolerance
    
    // Compute time-optimal parametrization with jerk constraints
    std::cout << "Computing time-optimal parametrization with jerk constraints..." << std::endl;
    
    ReturnCode ret = algo->computePathParametrization(0.0, 0.0, true);  // start_vel=0, end_vel=0, enable_jerk=true
    
    if (ret == ReturnCode::OK) {
        std::cout << "Success! Parametrization computed." << std::endl;
        
        // Get the parametrization data
        const auto& data = algo->getParameterizationData();
        
        // Create a spline parametrizer for smooth output
        parametrizer::Spline param(path, data.gridpoints, data.parametrization);
        
        // Sample the trajectory
        double T = param.getDuration();
        std::cout << "Total duration: " << T << " seconds" << std::endl;
        
        // Sample at a few time points
        std::cout << "\nTrajectory samples:" << std::endl;
        std::cout << "Time\tPosition\t\tVelocity\t\tAcceleration\t\tJerk" << std::endl;
        
        for (double t = 0; t <= T; t += T/10) {
            Vector pos = param.eval_single(t, 0);
            Vector vel = param.eval_single(t, 1);
            Vector acc = param.eval_single(t, 2);
            
            // Approximate jerk (3rd derivative)
            double dt = 0.001;
            Vector acc_next = param.eval_single(std::min(t + dt, T), 2);
            Vector jerk = (acc_next - acc) / dt;
            
            std::cout << t << "\t" 
                      << pos.transpose() << "\t" 
                      << vel.transpose() << "\t"
                      << acc.transpose() << "\t"
                      << jerk.transpose() << std::endl;
        }
        
        // Compare with standard TOPPRA (without jerk constraints)
        std::cout << "\n\nComparing with standard TOPPRA (no jerk constraints):" << std::endl;
        
        // Remove jerk constraint
        constraints.pop_back();
        
        // Create standard TOPPRA solver
        auto algo_standard = std::make_shared<algorithm::TOPPRA>(constraints, path);
        algo_standard->setN(100);
        
        ret = algo_standard->computePathParametrization(0.0, 0.0);
        
        if (ret == ReturnCode::OK) {
            const auto& data_standard = algo_standard->getParameterizationData();
            parametrizer::Spline param_standard(path, data_standard.gridpoints, data_standard.parametrization);
            
            double T_standard = param_standard.getDuration();
            std::cout << "Duration without jerk constraints: " << T_standard << " seconds" << std::endl;
            std::cout << "Difference: " << T - T_standard << " seconds (slower with jerk constraints)" << std::endl;
        }
        
    } else {
        std::cout << "Failed to compute parametrization. Error: " << algo->getErrorMessage() << std::endl;
    }
    
    return 0;
}