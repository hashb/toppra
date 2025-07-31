#ifndef TOPPRA_CONSTRAINT_JOINT_JERK_HPP
#define TOPPRA_CONSTRAINT_JOINT_JERK_HPP

#include <toppra/constraint.hpp>
#include <toppra/toppra.hpp>

namespace toppra {
namespace constraint {

/** \brief A third-order constraint on joint jerk.
 *
 * This class implements joint jerk constraints of the form:
 * \f[
 *     |q^{(3)}| \leq j_{max}
 * \f]
 *
 * The constraint requires three consecutive grid points (k, k+1, k+2) to
 * compute the jerk using finite differencing.
 *
 * \note This is a third-order constraint and requires special handling
 * in the path parametrization algorithm (TOPP3).
 */
class JointJerkConstraint : public LinearConstraint {
public:
  /** Constructor for joint jerk constraint.
   * 
   * \param jmax Vector of size dof. Maximum allowed jerk for each joint.
   *             Can be a scalar for uniform bounds across all joints.
   */
  JointJerkConstraint(const Vector& jmax);

  /** \brief Check if this is a third-order constraint.
   * 
   * Third-order constraints require special treatment in the algorithm
   * as they involve three consecutive grid points.
   */
  bool isThirdOrder() const { return true; }

  /** \brief Get the maximum jerk limits.
   */
  const Vector& getJerkLimits() const { return m_jmax; }

  std::ostream& print(std::ostream& os) const override;
  
  /** \brief Compute jerk constraint matrices for grid point k.
   *
   * Computes constraint of the form:
   * J0*x[k] + J1*x[k+1] + J2*x[k+2] <= jmax * dt[k]
   * 
   * This version returns the raw J matrices without time scaling.
   * For linearized constraints, use computeJerkConstraintLinearized.
   */
  void computeJerkConstraint(const GeometricPath& path,
                           int k,
                           const Vector& gridpoints,
                           Matrix& J,
                           Vector& h) const;
  
  /** \brief Compute linearized jerk constraint for TOPP3.
   *
   * Linearizes the constraint around current x values to handle
   * the nonlinear dt[k] term properly.
   * 
   * Returns constraint of the form:
   * A*[x[k], x[k+1], x[k+2]]^T <= b
   * 
   * Where the time dependency is linearized around x0_list values.
   */
  void computeJerkConstraintLinearized(const GeometricPath& path,
                                     int k,
                                     const Vector& gridpoints,
                                     const std::vector<value_type>& x0_list,
                                     Matrix& A,
                                     Vector& b) const;

protected:
  void computeParams_impl(const GeometricPath& path,
                         const Vector& gridpoints,
                         Vectors& a, Vectors& b, Vectors& c,
                         Matrices& F, Vectors& g,
                         Bounds& ubound, Bounds& xbound) override;

private:
  Vector m_jmax;  ///< Maximum jerk limits for each joint
};

}  // namespace constraint
}  // namespace toppra

#endif  // TOPPRA_CONSTRAINT_JOINT_JERK_HPP