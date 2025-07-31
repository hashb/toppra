#include <toppra/constraint/joint_jerk.hpp>
#include <toppra/geometric_path.hpp>
#include <algorithm>
#include <cmath>

namespace toppra {
namespace constraint {

JointJerkConstraint::JointJerkConstraint(const Vector& jmax)
    : LinearConstraint(0, 0, false, false, false),  // Will be set properly later
      m_jmax(jmax) {
  // For jerk constraints, we don't use the standard LinearConstraint format
  // as they involve three consecutive grid points
  // The actual constraint handling will be done in the TOPP3 algorithm
  if (m_jmax.size() == 0) {
    throw std::invalid_argument("Jerk limits must be non-empty");
  }
  if ((m_jmax.array() <= 0).any()) {
    throw std::invalid_argument("Jerk limits must be positive");
  }
}

void JointJerkConstraint::computeParams_impl(const GeometricPath& path,
                                           const Vector& gridpoints,
                                           Vectors& a, Vectors& b, Vectors& c,
                                           Matrices& F, Vectors& g,
                                           Bounds& ubound, Bounds& xbound) {
  // Third-order constraints are handled differently in TOPP3
  // This method is not used for jerk constraints
  // Instead, the TOPP3 algorithm will call computeJerkConstraint directly
  
  // Initialize empty parameters to satisfy the interface
  for (size_t i = 0; i < gridpoints.size(); ++i) {
    a[i] = Vector::Zero(path.dof());
    b[i] = Vector::Zero(path.dof());
    c[i] = Vector::Zero(path.dof());
    F[i] = Matrix::Zero(0, path.dof());
    g[i] = Vector::Zero(0);
  }
}

void JointJerkConstraint::computeJerkConstraint(const GeometricPath& path,
                                               int k,
                                               const Vector& gridpoints,
                                               Matrix& J,
                                               Vector& h) const {
  const int dof = path.dof();
  const double MIN_SDOT = 1e-5;
  
  // Get path derivatives at grid points
  Vector q_k = path.eval_single(gridpoints[k], 0);
  Vector dq_k = path.eval_single(gridpoints[k], 1);
  Vector ddq_k = path.eval_single(gridpoints[k], 2);
  
  Vector q_k1 = path.eval_single(gridpoints[k+1], 0);
  Vector dq_k1 = path.eval_single(gridpoints[k+1], 1);
  Vector ddq_k1 = path.eval_single(gridpoints[k+1], 2);
  
  if (k + 2 < gridpoints.size()) {
    Vector q_k2 = path.eval_single(gridpoints[k+2], 0);
    Vector dq_k2 = path.eval_single(gridpoints[k+2], 1);
    Vector ddq_k2 = path.eval_single(gridpoints[k+2], 2);
  }
  
  // Compute grid spacing
  double ds_k = gridpoints[k+1] - gridpoints[k];
  double ds_k1 = (k + 2 < gridpoints.size()) ? gridpoints[k+2] - gridpoints[k+1] : ds_k;
  
  // Compute J0, J1, J2 coefficients for jerk approximation
  // dddq[k] = (ddq[k+1] - ddq[k]) / dt[k]
  // ddq[k+1] - ddq[k] = J0*x[k] + J1*x[k+1] + J2*x[k+2]
  Vector J0 = dq_k / (2.0 * ds_k) - ddq_k;
  Vector J1 = -dq_k1 / (2.0 * ds_k1) - dq_k / (2.0 * ds_k) + ddq_k1;
  Vector J2 = dq_k1 / (2.0 * ds_k1);
  
  // Jerk constraint: -jmax <= J0*x[k] + J1*x[k+1] + J2*x[k+2] / dt[k] <= jmax
  // We need to handle dt[k] which depends on x values
  // This will be linearized in TOPP3 using trust regions
  
  // Set up constraint matrices
  J = Matrix::Zero(2 * dof, 3);
  h = Vector::Zero(2 * dof);
  
  // Upper bounds: J0*x[k] + J1*x[k+1] + J2*x[k+2] <= jmax * dt[k]
  J.block(0, 0, dof, 1) = J0;
  J.block(0, 1, dof, 1) = J1;
  J.block(0, 2, dof, 1) = J2;
  
  // Lower bounds: -J0*x[k] - J1*x[k+1] - J2*x[k+2] <= jmax * dt[k]
  J.block(dof, 0, dof, 1) = -J0;
  J.block(dof, 1, dof, 1) = -J1;
  J.block(dof, 2, dof, 1) = -J2;
  
  // The right-hand side will be computed in TOPP3 based on current x values
  // For now, we store the jerk limits
  h.head(dof) = m_jmax;
  h.tail(dof) = m_jmax;
}

void JointJerkConstraint::computeJerkConstraintLinearized(const GeometricPath& path,
                                                        int k,
                                                        const Vector& gridpoints,
                                                        const std::vector<value_type>& x0_list,
                                                        Matrix& A,
                                                        Vector& b) const {
  const int dof = path.dof();
  const value_type MIN_SDOT = 1e-5;
  
  // Get path derivatives
  Vector dq_k = path.eval_single(gridpoints[k], 1);
  Vector ddq_k = path.eval_single(gridpoints[k], 2);
  Vector dq_k1 = path.eval_single(gridpoints[k+1], 1);
  Vector ddq_k1 = path.eval_single(gridpoints[k+1], 2);
  
  // Grid spacing
  value_type ds_k = gridpoints[k+1] - gridpoints[k];
  value_type ds_k1 = gridpoints[k+2] - gridpoints[k+1];
  
  // Compute J0, J1, J2 coefficients
  // ddq[k+1] - ddq[k] = J0*x[k] + J1*x[k+1] + J2*x[k+2]
  Vector J0 = dq_k / (2.0 * ds_k) - ddq_k;
  Vector J1 = -dq_k1 / (2.0 * ds_k1) - dq_k / (2.0 * ds_k) + ddq_k1;
  Vector J2 = dq_k1 / (2.0 * ds_k1);
  
  // Current velocities
  value_type vk0 = std::sqrt(std::max(MIN_SDOT, x0_list[k]));
  value_type vk1 = std::sqrt(std::max(MIN_SDOT, x0_list[k+1]));
  value_type vk2 = std::sqrt(std::max(MIN_SDOT, x0_list[k+2]));
  
  // Linearize dt[k] = ds_k/(vk0 + vk1) + ds_k1/(vk1 + vk2)
  // dt[k] ≈ hbar + dh0*x[k] + dh1*x[k+1] + dh2*x[k+2]
  value_type dh0, dh1, dh2, hbar;
  
  // Handle boundary cases
  if (k == 0) {
    // At start, x[0] is fixed
    dh0 = 0.0;
    dh1 = (-ds_k / std::pow(vk0 + vk1, 2) - ds_k1 / std::pow(vk1 + vk2, 2)) * 0.5 / vk1;
    dh2 = -ds_k1 / std::pow(vk1 + vk2, 2) * 0.5 / vk2;
  } else if (k == x0_list.size() - 3) {
    // Near end, x[N-1] is fixed
    dh0 = -ds_k / std::pow(vk0 + vk1, 2) * 0.5 / vk0;
    dh1 = (-ds_k / std::pow(vk0 + vk1, 2) - ds_k1 / std::pow(vk1 + vk2, 2)) * 0.5 / vk1;
    dh2 = 0.0;
  } else {
    // General case
    dh0 = -ds_k / std::pow(vk0 + vk1, 2) * 0.5 / vk0;
    dh1 = (-ds_k / std::pow(vk0 + vk1, 2) - ds_k1 / std::pow(vk1 + vk2, 2)) * 0.5 / vk1;
    dh2 = -ds_k1 / std::pow(vk1 + vk2, 2) * 0.5 / vk2;
  }
  
  // Compute hbar (linearization point)
  hbar = ds_k / (vk0 + vk1) + ds_k1 / (vk1 + vk2);
  hbar = hbar - dh0 * x0_list[k] - dh1 * x0_list[k+1] - dh2 * x0_list[k+2];
  
  // Build constraint matrices
  // Jerk constraint: |J0*x[k] + J1*x[k+1] + J2*x[k+2]| <= jmax * dt[k]
  // With linearized dt: |J0*x[k] + J1*x[k+1] + J2*x[k+2]| <= jmax * (hbar + dh0*x[k] + dh1*x[k+1] + dh2*x[k+2])
  // Rearranging: -(J0 + jmax*dh0)*x[k] - (J1 + jmax*dh1)*x[k+1] - (J2 + jmax*dh2)*x[k+2] <= jmax*hbar
  //              (J0 - jmax*dh0)*x[k] + (J1 - jmax*dh1)*x[k+1] + (J2 - jmax*dh2)*x[k+2] <= jmax*hbar
  
  A = Matrix::Zero(2 * dof, 3);
  b = Vector::Zero(2 * dof);
  
  // Upper bound
  for (int i = 0; i < dof; ++i) {
    A(i, 0) = -(J0[i] + m_jmax[i] * dh0);
    A(i, 1) = -(J1[i] + m_jmax[i] * dh1);
    A(i, 2) = -(J2[i] + m_jmax[i] * dh2);
    b[i] = m_jmax[i] * hbar;
  }
  
  // Lower bound
  for (int i = 0; i < dof; ++i) {
    A(dof + i, 0) = J0[i] - m_jmax[i] * dh0;
    A(dof + i, 1) = J1[i] - m_jmax[i] * dh1;
    A(dof + i, 2) = J2[i] - m_jmax[i] * dh2;
    b[dof + i] = m_jmax[i] * hbar;
  }
}

std::ostream& JointJerkConstraint::print(std::ostream& os) const {
  os << "JointJerkConstraint(jmax=" << m_jmax.transpose() << ")";
  return os;
}

}  // namespace constraint
}  // namespace toppra