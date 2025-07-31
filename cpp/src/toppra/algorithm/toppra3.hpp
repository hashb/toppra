#ifndef TOPPRA_ALGORITHM_TOPPRA3_HPP
#define TOPPRA_ALGORITHM_TOPPRA3_HPP

#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/joint_jerk.hpp>
#include <memory>

namespace toppra {
namespace algorithm {

/** \brief Time-Optimal Path Parameterization with jerk constraints (TOPP3).
 *
 * This class extends TOPPRA to handle third-order (jerk) constraints using
 * an iterative refinement approach. The algorithm works in two phases:
 * 
 * 1. TOPPRA: Computes velocity profile considering up to 2nd order constraints
 * 2. TOPP3: Refines the solution considering 3rd order (jerk) constraints
 *
 * The TOPP3 phase uses a trust-region based optimization to handle the
 * nonlinear nature of jerk constraints while maintaining feasibility.
 */
class TOPPRA3 : public TOPPRA {
public:
  /** \brief Construct TOPPRA3 solver.
   *
   * \param constraints List of constraints (including jerk constraints)
   * \param path The geometric path
   */
  TOPPRA3(LinearConstraintPtrs constraints, const GeometricPathPtr& path);

  /** \brief Compute path parametrization with jerk constraints.
   *
   * \param vel_start Starting velocity (default: 0)
   * \param vel_end Ending velocity (default: 0)
   * \param enable_jerk Whether to enable jerk constraint handling (default: true)
   * \return Return code
   */
  ReturnCode computePathParametrization(value_type vel_start = 0,
                                      value_type vel_end = 0,
                                      bool enable_jerk = true);

  /** \brief Set TOPP3 solver parameters.
   *
   * \param max_iterations Maximum iterations for TOPP3 refinement
   * \param trust_region_size Initial trust region size (0 < alpha <= 1)
   * \param tolerance Convergence tolerance
   */
  void setTOPP3Parameters(int max_iterations = 10,
                         value_type trust_region_size = 0.5,
                         value_type tolerance = 1e-6) {
    m_topp3_max_iter = max_iterations;
    m_topp3_trust_region = trust_region_size;
    m_topp3_tolerance = tolerance;
  }

protected:
  /** \brief Run TOPP3 refinement for jerk constraints.
   *
   * This method iteratively refines the velocity profile to satisfy
   * jerk constraints using a trust-region approach.
   *
   * \param x0_list Initial velocity profile from TOPPRA
   * \param idx_start Starting index for refinement (default: 0)
   * \return true if refinement succeeded
   */
  bool solveTOPP3(std::vector<value_type>& x0_list, int idx_start = 0);

  /** \brief Build constraint matrices for TOPP3 optimization.
   *
   * Combines first, second, and third order constraints for the
   * optimization subproblem.
   *
   * \param x0_list Current velocity profile
   * \param k Starting waypoint index
   * \param h Number of waypoints to optimize
   * \param alpha Trust region size
   * \return Constraint data for each order
   */
  struct ConstraintData {
    Matrix A;
    Vector b;
  };
  
  std::vector<ConstraintData> buildTOPP3Constraints(
      const std::vector<value_type>& x0_list,
      int k, int h, value_type alpha);

  /** \brief Compute jerk at grid point k.
   *
   * \param k Grid point index
   * \param x_list Current velocity profile
   * \return Jerk vector at grid point k
   */
  Vector computeJerk(int k, const std::vector<value_type>& x_list) const;

  /** \brief Check if jerk constraints are satisfied.
   *
   * \param x_list Velocity profile to check
   * \return Maximum jerk constraint violation (0 if all satisfied)
   */
  value_type checkJerkLimits(const std::vector<value_type>& x_list) const;

private:
  // Extract jerk constraints from the constraint list
  void extractJerkConstraints();

  // TOPP3 solver parameters
  int m_topp3_max_iter = 10;
  value_type m_topp3_trust_region = 0.5;
  value_type m_topp3_tolerance = 1e-6;

  // Jerk constraints
  std::vector<std::shared_ptr<constraint::JointJerkConstraint>> m_jerk_constraints;
  bool m_has_jerk_constraints = false;
  
  // Combined jerk limits (minimum across all jerk constraints)
  Vector m_jmax;
};

}  // namespace algorithm
}  // namespace toppra

#endif  // TOPPRA_ALGORITHM_TOPPRA3_HPP