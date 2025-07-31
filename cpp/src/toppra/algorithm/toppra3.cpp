#include <toppra/algorithm/toppra3.hpp>
#include <toppra/solver.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace toppra {
namespace algorithm {

TOPPRA3::TOPPRA3(LinearConstraintPtrs constraints, const GeometricPathPtr& path)
    : TOPPRA(constraints, path) {
  extractJerkConstraints();
}

void TOPPRA3::extractJerkConstraints() {
  m_jerk_constraints.clear();
  
  // Extract jerk constraints from the constraint list
  for (const auto& constraint : m_constraints) {
    auto jerk_constraint = std::dynamic_pointer_cast<constraint::JointJerkConstraint>(constraint);
    if (jerk_constraint) {
      m_jerk_constraints.push_back(jerk_constraint);
      m_has_jerk_constraints = true;
    }
  }
  
  if (m_has_jerk_constraints) {
    // Compute combined jerk limits (minimum across all constraints)
    m_jmax = m_jerk_constraints[0]->getJerkLimits();
    for (size_t i = 1; i < m_jerk_constraints.size(); ++i) {
      m_jmax = m_jmax.cwiseMin(m_jerk_constraints[i]->getJerkLimits());
    }
  }
}

ReturnCode TOPPRA3::computePathParametrization(value_type vel_start,
                                              value_type vel_end,
                                              bool enable_jerk) {
  // First, run standard TOPPRA without jerk constraints
  ReturnCode ret = TOPPRA::computePathParametrization(vel_start, vel_end);
  
  if (ret != ReturnCode::OK) {
    return ret;
  }
  
  // If jerk constraints exist and are enabled, run TOPP3 refinement
  if (enable_jerk && m_has_jerk_constraints) {
    // Convert parametrization to vector for TOPP3
    std::vector<value_type> x_list(m_data.parametrization.data(),
                                   m_data.parametrization.data() + m_data.parametrization.size());
    
    bool topp3_success = solveTOPP3(x_list);
    
    if (!topp3_success) {
      m_errorStream << "TOPP3 refinement failed to converge\n";
      return ReturnCode::ERR_FAIL_FORWARD_PASS;
    }
    
    // Update parametrization with refined values
    for (size_t i = 0; i < x_list.size(); ++i) {
      m_data.parametrization[i] = x_list[i];
    }
  }
  
  return ReturnCode::OK;
}

bool TOPPRA3::solveTOPP3(std::vector<value_type>& x0_list, int idx_start) {
  const value_type MIN_SDOT = 1e-5;
  const int N = x0_list.size() - 1;
  
  // Window size for local optimization
  const int window_size = std::min(10, N);
  
  value_type max_jerk_violation = checkJerkLimits(x0_list);
  
  if (max_jerk_violation <= m_topp3_tolerance) {
    return true;  // Already satisfies jerk constraints
  }
  
  std::vector<value_type> x_best = x0_list;
  value_type best_violation = max_jerk_violation;
  
  // Iterative refinement
  for (int iter = 0; iter < m_topp3_max_iter; ++iter) {
    bool improved = false;
    
    // Slide window through the trajectory
    for (int k = idx_start; k < N - 1; k += window_size / 2) {
      int h = std::min(window_size, N - k);
      if (h < 2) continue;  // Need at least 2 points
      
      // Build constraints for local optimization
      auto constraints = buildTOPP3Constraints(x0_list, k, h, m_topp3_trust_region);
      
      // Set up LP problem
      // Variables: x[k+1], ..., x[k+h]
      // Minimize: sum of x values (time-optimal)
      Vector c = Vector::Ones(h);
      
      // Combine all constraints
      Matrix A_total;
      Vector b_total;
      
      // Stack constraint matrices
      int total_rows = 0;
      for (const auto& constr : constraints) {
        total_rows += constr.A.rows();
      }
      
      A_total = Matrix::Zero(total_rows, h);
      b_total = Vector::Zero(total_rows);
      
      int row_offset = 0;
      for (const auto& constr : constraints) {
        A_total.block(row_offset, 0, constr.A.rows(), h) = constr.A;
        b_total.segment(row_offset, constr.A.rows()) = constr.b;
        row_offset += constr.A.rows();
      }
      
      // Solve LP
      Vector x_opt(h);
      bool solve_success = false;
      
      if (m_solver) {
        // Use bounds [MIN_SDOT, inf] for each variable
        Bounds var_bounds(h, 2);
        for (int i = 0; i < h; ++i) {
          var_bounds(i, 0) = MIN_SDOT * MIN_SDOT;  // x = sdot^2
          var_bounds(i, 1) = std::numeric_limits<value_type>::infinity();
        }
        
        // Solve the LP problem
        auto lp_result = m_solver->solveLPbasic(c, A_total, b_total, var_bounds);
        
        if (lp_result.feasible) {
          x_opt = lp_result.feasible_result;
          solve_success = true;
        }
      }
      
      if (solve_success) {
        // Update solution
        for (int i = 0; i < h; ++i) {
          x0_list[k + 1 + i] = x_opt[i];
        }
        
        // Check if improved
        value_type new_violation = checkJerkLimits(x0_list);
        if (new_violation < best_violation) {
          x_best = x0_list;
          best_violation = new_violation;
          improved = true;
        }
      }
    }
    
    // Check convergence
    if (best_violation <= m_topp3_tolerance) {
      x0_list = x_best;
      return true;
    }
    
    if (!improved) {
      // Reduce trust region if no improvement
      m_topp3_trust_region *= 0.5;
      if (m_topp3_trust_region < 0.1) {
        break;  // Trust region too small
      }
    }
    
    x0_list = x_best;
    max_jerk_violation = best_violation;
  }
  
  // Return best solution found even if not fully converged
  x0_list = x_best;
  return best_violation < max_jerk_violation * 0.9;  // At least 10% improvement
}

std::vector<TOPPRA3::ConstraintData> TOPPRA3::buildTOPP3Constraints(
    const std::vector<value_type>& x0_list,
    int k, int h, value_type alpha) {
  
  std::vector<ConstraintData> result;
  const int dof = m_path->dof();
  
  // 1. First-order constraints (velocity limits)
  // For each grid point k+1, ..., k+h, we need velocity constraints
  {
    ConstraintData vel_constr;
    int total_vel_constraints = 0;
    
    // Count constraints from non-jerk constraints
    for (const auto& constraint : m_constraints) {
      if (!std::dynamic_pointer_cast<constraint::JointJerkConstraint>(constraint)) {
        if (constraint->hasLinearInequalities()) {
          total_vel_constraints += constraint->nbConstraints() * h;
        }
        if (constraint->hasXbounds()) {
          total_vel_constraints += 2 * h;  // Upper and lower bounds
        }
      }
    }
    
    if (total_vel_constraints > 0) {
      vel_constr.A = Matrix::Zero(total_vel_constraints, h);
      vel_constr.b = Vector::Zero(total_vel_constraints);
      
      int row_offset = 0;
      
      // For each grid point in the window
      for (int i = 0; i < h; ++i) {
        int grid_idx = k + 1 + i;
        
        // Apply constraints at this grid point
        for (const auto& constraint : m_constraints) {
          if (!std::dynamic_pointer_cast<constraint::JointJerkConstraint>(constraint)) {
            // Get constraint parameters at this grid point
            Vectors a(1), b(1), c(1), g(1);
            Matrices F(1);
            Bounds ubound(1), xbound(1);
            
            Vector grid_pt(1);
            grid_pt[0] = m_data.gridpoints[grid_idx];
            
            constraint->allocateParams(1, a, b, c, F, g, ubound, xbound);
            constraint->computeParams(*m_path, grid_pt, a, b, c, F, g, ubound, xbound);
            
            // Add velocity-dependent constraints
            if (constraint->hasLinearInequalities() && F[0].rows() > 0) {
              // F[0] * (b[0] * x) <= g[0]
              // This becomes: F[0] * b[0] * x[i] <= g[0]
              Vector Fb = F[0] * b[0];
              vel_constr.A.block(row_offset, i, Fb.size(), 1) = Fb;
              vel_constr.b.segment(row_offset, Fb.size()) = g[0];
              row_offset += Fb.size();
            }
            
            // Add x bounds
            if (constraint->hasXbounds()) {
              // x >= xbound[0](0)
              vel_constr.A(row_offset, i) = -1.0;
              vel_constr.b(row_offset) = -xbound[0](0);
              row_offset++;
              
              // x <= xbound[0](1)
              vel_constr.A(row_offset, i) = 1.0;
              vel_constr.b(row_offset) = xbound[0](1);
              row_offset++;
            }
          }
        }
      }
      
      // Resize to actual size used
      if (row_offset < total_vel_constraints) {
        vel_constr.A.conservativeResize(row_offset, h);
        vel_constr.b.conservativeResize(row_offset);
      }
      
      if (row_offset > 0) {
        result.push_back(vel_constr);
      }
    }
  }
  
  // 2. Second-order constraints (acceleration/torque limits)
  // These couple consecutive grid points
  {
    ConstraintData acc_constr;
    int total_acc_constraints = 0;
    
    // Count constraints
    for (const auto& constraint : m_constraints) {
      if (!std::dynamic_pointer_cast<constraint::JointJerkConstraint>(constraint)) {
        if (constraint->hasLinearInequalities()) {
          total_acc_constraints += constraint->nbConstraints() * h;
        }
      }
    }
    
    if (total_acc_constraints > 0) {
      acc_constr.A = Matrix::Zero(total_acc_constraints, h);
      acc_constr.b = Vector::Zero(total_acc_constraints);
      
      int row_offset = 0;
      
      // For each interval in the window
      for (int i = 0; i < h; ++i) {
        int grid_idx = k + i;
        
        // Apply constraints that involve grid_idx and grid_idx+1
        for (const auto& constraint : m_constraints) {
          if (!std::dynamic_pointer_cast<constraint::JointJerkConstraint>(constraint)) {
            // Get constraint parameters
            Vectors a(2), b(2), c(2), g(2);
            Matrices F(2);
            Bounds ubound(2), xbound(2);
            
            Vector grid_pts(2);
            grid_pts[0] = m_data.gridpoints[grid_idx];
            grid_pts[1] = m_data.gridpoints[grid_idx + 1];
            
            constraint->allocateParams(2, a, b, c, F, g, ubound, xbound);
            constraint->computeParams(*m_path, grid_pts, a, b, c, F, g, ubound, xbound);
            
            // Compute second-order constraint coefficients
            // u = (x[k+1] - x[k]) / (2 * ds)
            value_type ds = grid_pts[1] - grid_pts[0];
            
            if (constraint->hasLinearInequalities() && F[0].rows() > 0) {
              // At grid_idx: F[0] * (a[0] * u + b[0] * x[k] + c[0]) <= g[0]
              // u = (x[k+1] - x[k]) / (2 * ds)
              // So: F[0] * (a[0]/2ds * x[k+1] + (b[0] - a[0]/2ds) * x[k] + c[0]) <= g[0]
              
              Vector coeff_k = F[0] * (b[0] - a[0] / (2.0 * ds));
              Vector coeff_k1 = F[0] * (a[0] / (2.0 * ds));
              Vector rhs = g[0] - F[0] * c[0];
              
              if (i == 0) {
                // x[k] is fixed
                value_type xk = x0_list[k];
                acc_constr.A.block(row_offset, 0, coeff_k1.size(), 1) = coeff_k1;
                acc_constr.b.segment(row_offset, coeff_k1.size()) = rhs - coeff_k * xk;
              } else if (i == h - 1 && grid_idx + 1 == x0_list.size() - 1) {
                // x[k+h+1] is fixed
                value_type xkh1 = x0_list[grid_idx + 1];
                acc_constr.A.block(row_offset, i - 1, coeff_k.size(), 1) = coeff_k;
                acc_constr.b.segment(row_offset, coeff_k.size()) = rhs - coeff_k1 * xkh1;
              } else {
                // Both are variables
                acc_constr.A.block(row_offset, std::max(0, i - 1), coeff_k.size(), 1) = coeff_k;
                if (i < h) {
                  acc_constr.A.block(row_offset, i, coeff_k1.size(), 1) = coeff_k1;
                }
                acc_constr.b.segment(row_offset, coeff_k.size()) = rhs;
              }
              row_offset += coeff_k.size();
            }
          }
        }
      }
      
      // Resize to actual size used
      if (row_offset < total_acc_constraints) {
        acc_constr.A.conservativeResize(row_offset, h);
        acc_constr.b.conservativeResize(row_offset);
      }
      
      if (row_offset > 0) {
        result.push_back(acc_constr);
      }
    }
  }
  
  // 3. Third-order constraints (jerk limits)
  if (m_has_jerk_constraints && k + h + 1 < x0_list.size()) {
    ConstraintData jerk_constr;
    
    // Number of jerk constraints
    int n_jerk_constr = 0;
    if (k > 0) n_jerk_constr += 2 * dof;  // Constraint at k-1
    n_jerk_constr += (h - 1) * 2 * dof;   // Constraints at k, ..., k+h-2
    
    jerk_constr.A = Matrix::Zero(n_jerk_constr, h);
    jerk_constr.b = Vector::Zero(n_jerk_constr);
    
    int row_idx = 0;
    
    // Handle constraint at k-1 if applicable
    if (k > 0) {
      Matrix A_jerk;
      Vector b_jerk;
      m_jerk_constraints[0]->computeJerkConstraintLinearized(*m_path, k-1, m_data.gridpoints, x0_list, A_jerk, b_jerk);
      
      // Only x[k+1] is a variable, x[k-1] and x[k] are fixed
      value_type xkm1 = x0_list[k-1];
      value_type xk = x0_list[k];
      
      jerk_constr.A.block(row_idx, 0, 2*dof, 1) = A_jerk.col(2);
      jerk_constr.b.segment(row_idx, 2*dof) = b_jerk - A_jerk.col(0) * xkm1 - A_jerk.col(1) * xk;
      row_idx += 2 * dof;
    }
    
    // Handle constraints at k, ..., k+h-2
    for (int i = 0; i < h - 1; ++i) {
      if (k + i + 2 >= x0_list.size()) break;
      
      Matrix A_jerk;
      Vector b_jerk;
      m_jerk_constraints[0]->computeJerkConstraintLinearized(*m_path, k+i, m_data.gridpoints, x0_list, A_jerk, b_jerk);
      
      if (i == 0) {
        // x[k] is fixed
        value_type xk = x0_list[k];
        jerk_constr.A.block(row_idx, 0, 2*dof, 2) = A_jerk.rightCols(2);
        jerk_constr.b.segment(row_idx, 2*dof) = b_jerk - A_jerk.col(0) * xk;
      } else if (i == h - 2 && k + h + 1 == x0_list.size() - 1) {
        // x[k+h+1] is fixed (last point)
        value_type xkh1 = x0_list[k + h + 1];
        jerk_constr.A.block(row_idx, i-1, 2*dof, 2) = A_jerk.leftCols(2);
        jerk_constr.b.segment(row_idx, 2*dof) = b_jerk - A_jerk.col(2) * xkh1;
      } else {
        // All three x values are variables
        jerk_constr.A.block(row_idx, i-1, 2*dof, 3) = A_jerk;
        jerk_constr.b.segment(row_idx, 2*dof) = b_jerk;
      }
      row_idx += 2 * dof;
    }
    
    // Add boundary conditions for jerk constraints
    if (k == 0 && h > 0) {
      // At start: assume zero initial acceleration
      // qddot[0] / (0.5*dt[0]) <= jmax
      // (dq[0]/2/ds/ds)*x[1]^(3/2) <= jmax
      value_type ds = m_data.gridpoints[1] - m_data.gridpoints[0];
      Vector dq0_scaled = m_path->eval_single(m_data.gridpoints[0], 1) / (2.0 * ds * ds);
      
      // Find the most restrictive constraint
      value_type max_x1 = std::numeric_limits<value_type>::infinity();
      for (int i = 0; i < dof; ++i) {
        if (std::abs(dq0_scaled[i]) > 1e-10) {
          value_type limit = std::pow(m_jmax[i] / std::abs(dq0_scaled[i]), 2.0/3.0);
          max_x1 = std::min(max_x1, limit);
        }
      }
      
      // Add constraint: x[1] <= max_x1
      if (max_x1 < std::numeric_limits<value_type>::infinity()) {
        Matrix A_start = Matrix::Zero(1, h);
        Vector b_start = Vector::Zero(1);
        A_start(0, 0) = 1.0;  // x[k+1] is the first variable
        b_start[0] = max_x1;
        
        // Append to existing constraints
        Matrix A_combined = Matrix::Zero(jerk_constr.A.rows() + 1, h);
        Vector b_combined = Vector::Zero(jerk_constr.b.size() + 1);
        A_combined.topRows(jerk_constr.A.rows()) = jerk_constr.A;
        b_combined.head(jerk_constr.b.size()) = jerk_constr.b;
        A_combined.bottomRows(1) = A_start;
        b_combined.tail(1) = b_start;
        
        jerk_constr.A = A_combined;
        jerk_constr.b = b_combined;
      }
    }
    
    if (k + h + 1 == x0_list.size() - 1 && h > 0) {
      // At end: assume zero final acceleration
      // qddot[N-2] / (0.5*dt[N-2]) <= jmax
      // (ddq[N-2] - dq[N-2]/2/ds)/ds * x[N-2]^(3/2) <= jmax
      int N = x0_list.size() - 1;
      value_type ds = m_data.gridpoints[N] - m_data.gridpoints[N-1];
      Vector ddq_N2 = m_path->eval_single(m_data.gridpoints[N-1], 2);
      Vector dq_N2 = m_path->eval_single(m_data.gridpoints[N-1], 1);
      Vector dq_scaled = (ddq_N2 - dq_N2 / (2.0 * ds)) / ds;
      
      // Find the most restrictive constraint
      value_type max_xN2 = std::numeric_limits<value_type>::infinity();
      for (int i = 0; i < dof; ++i) {
        if (std::abs(dq_scaled[i]) > 1e-10) {
          value_type limit = std::pow(m_jmax[i] / std::abs(dq_scaled[i]), 2.0/3.0);
          max_xN2 = std::min(max_xN2, limit);
        }
      }
      
      // Add constraint: x[N-2] <= max_xN2 (which is x[k+h] in local indexing)
      if (max_xN2 < std::numeric_limits<value_type>::infinity() && h > 0) {
        Matrix A_end = Matrix::Zero(1, h);
        Vector b_end = Vector::Zero(1);
        A_end(0, h-1) = 1.0;  // Last variable
        b_end[0] = max_xN2;
        
        // Append to existing constraints
        Matrix A_combined = Matrix::Zero(jerk_constr.A.rows() + 1, h);
        Vector b_combined = Vector::Zero(jerk_constr.b.size() + 1);
        A_combined.topRows(jerk_constr.A.rows()) = jerk_constr.A;
        b_combined.head(jerk_constr.b.size()) = jerk_constr.b;
        A_combined.bottomRows(1) = A_end;
        b_combined.tail(1) = b_end;
        
        jerk_constr.A = A_combined;
        jerk_constr.b = b_combined;
      }
    }
    
    result.push_back(jerk_constr);
  }
  
  // 4. Trust region constraints
  if (alpha > 0) {
    ConstraintData trust_constr;
    trust_constr.A = Matrix::Zero(2 * h, h);
    trust_constr.b = Vector::Zero(2 * h);
    
    for (int i = 0; i < h; ++i) {
      value_type r = alpha * x0_list[k + 1 + i];
      
      // x[k+1+i] <= x0[k+1+i] + r
      trust_constr.A(i, i) = 1.0;
      trust_constr.b(i) = x0_list[k + 1 + i] + r;
      
      // -x[k+1+i] <= -x0[k+1+i] + r
      trust_constr.A(h + i, i) = -1.0;
      trust_constr.b(h + i) = -x0_list[k + 1 + i] + r;
    }
    
    result.push_back(trust_constr);
  }
  
  return result;
}

Vector TOPPRA3::computeJerk(int k, const std::vector<value_type>& x_list) const {
  const value_type MIN_SDOT = 1e-5;
  
  if (k < 0 || k >= x_list.size() - 2) {
    return Vector::Zero(m_path->dof());
  }
  
  // Get path derivatives
  Vector dq_k = m_path->eval_single(m_data.gridpoints[k], 1);
  Vector ddq_k = m_path->eval_single(m_data.gridpoints[k], 2);
  Vector dq_k1 = m_path->eval_single(m_data.gridpoints[k+1], 1);
  Vector ddq_k1 = m_path->eval_single(m_data.gridpoints[k+1], 2);
  
  // Grid spacing
  value_type ds_k = m_data.gridpoints[k+1] - m_data.gridpoints[k];
  value_type ds_k1 = m_data.gridpoints[k+2] - m_data.gridpoints[k+1];
  
  // Compute velocities
  value_type vk0 = std::sqrt(std::max(MIN_SDOT * MIN_SDOT, x_list[k]));
  value_type vk1 = std::sqrt(std::max(MIN_SDOT * MIN_SDOT, x_list[k+1]));
  value_type vk2 = std::sqrt(std::max(MIN_SDOT * MIN_SDOT, x_list[k+2]));
  
  // Time interval
  value_type dt = ds_k1 / (vk2 + vk1) + ds_k / (vk1 + vk0);
  
  // Jerk coefficients
  Vector J0 = dq_k / (2.0 * ds_k) - ddq_k;
  Vector J1 = -dq_k1 / (2.0 * ds_k1) - dq_k / (2.0 * ds_k) + ddq_k1;
  Vector J2 = dq_k1 / (2.0 * ds_k1);
  
  // Compute jerk
  return (J0 * x_list[k] + J1 * x_list[k+1] + J2 * x_list[k+2]) / dt;
}

value_type TOPPRA3::checkJerkLimits(const std::vector<value_type>& x_list) const {
  if (!m_has_jerk_constraints) {
    return 0.0;
  }
  
  value_type max_violation = 0.0;
  
  // Check jerk at each grid point
  for (int k = 0; k < x_list.size() - 2; ++k) {
    Vector jerk = computeJerk(k, x_list);
    
    // Check against limits
    for (int i = 0; i < jerk.size(); ++i) {
      value_type violation = std::abs(jerk[i]) / m_jmax[i] - 1.0;
      if (violation > 0) {
        max_violation = std::max(max_violation, violation);
      }
    }
  }
  
  // Check boundary conditions
  // At start: assume zero initial acceleration
  if (x_list.size() > 1) {
    Vector dq0 = m_path->eval_single(m_data.gridpoints[0], 1);
    value_type ds0 = m_data.gridpoints[1] - m_data.gridpoints[0];
    value_type v1 = std::sqrt(std::max(1e-10, x_list[1]));
    
    Vector jerk0 = dq0 * v1 / (2.0 * ds0 * ds0);
    for (int i = 0; i < jerk0.size(); ++i) {
      value_type violation = std::abs(jerk0[i]) / m_jmax[i] - 1.0;
      if (violation > 0) {
        max_violation = std::max(max_violation, violation);
      }
    }
  }
  
  // At end: assume zero final acceleration
  int N = x_list.size() - 1;
  if (N > 0) {
    Vector dqN = m_path->eval_single(m_data.gridpoints[N], 1);
    Vector ddqN = m_path->eval_single(m_data.gridpoints[N], 2);
    value_type dsN = m_data.gridpoints[N] - m_data.gridpoints[N-1];
    value_type vN = std::sqrt(std::max(1e-10, x_list[N]));
    value_type vN1 = std::sqrt(std::max(1e-10, x_list[N-1]));
    
    Vector jerkN = -(ddqN - dqN / (2.0 * dsN)) * (vN + vN1) / (2.0 * dsN);
    for (int i = 0; i < jerkN.size(); ++i) {
      value_type violation = std::abs(jerkN[i]) / m_jmax[i] - 1.0;
      if (violation > 0) {
        max_violation = std::max(max_violation, violation);
      }
    }
  }
  
  return max_violation;
}

}  // namespace algorithm
}  // namespace toppra