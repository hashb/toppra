#include "toppra/user_command.hpp"
#include "math/lp_solver.hpp"

class LPSolver;
class TrajectoryManager;
class Clock;

struct InequalData {
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
};
typedef std::array<InequalData, 3> InequalDataList;
// let kk:k+1, kpre:k-1

/**
 * @brief Time-Optimal Path Parameterization (TOPP) solver with jerk constraints
 *
 * This class implements a Time-Optimal Path Parameterization solver that finds the
 * optimal velocity profile along a predefined geometric path while respecting:
 * - Joint velocity limits
 * - Joint acceleration limits
 * - Joint torque limits
 * - Joint jerk limits
 * - End-effector linear velocity/acceleration limits
 * - Grasping force constraints for suction grippers
 *
 * The solver uses a two-phase approach:
 * 1. TOPPRA (Time-Optimal Path Parameterization with Reachability Analysis):
 *    Computes velocity profile considering up to 2nd order constraints
 * 2. TOPP3:
 *    Refines the solution considering 3rd order (jerk) constraints
 *
 * The optimization is formulated as a Linear Programming (LP) problem at each waypoint,
 * solving for the squared path velocity (s_dot^2).
 *
 * Key variables:
 * s: Path parameter (0 to 1)
 * x: Squared path velocity (s_dot^2)
 * u: Path acceleration (s_ddot)
 */
class ToptSolver {
 public:
  /**
   * @brief Constructs a TOPP solver
   * @param dim Number of robot joints/actuators
   */
  ToptSolver(int dim);
  ~ToptSolver() {};

  /**
   * @brief Main solve method for time-optimal trajectory
   *
   * @param sysdata System data containing path and constraints
   * @param gripdata Gripper data for suction force constraints
   * @param traj Output trajectory manager to store solution
   * @param thirdorder If true, includes jerk constraints (default: true)
   * @return true if solution found, false otherwise
   *
   * Algorithm:
   * 1. Initialize system with path and constraints
   * 2. Run TOPPRA to get initial velocity profile
   * 3. If thirdorder=true, run TOPP3 to refine with jerk constraints
   * 4. Generate final time parameterization
   */
  bool solve(const SYSTEM_DATA& sysdata,
             TrajectoryManager* traj, bool thirdorder = true);

  /**
   * @brief Resolves trajectory from a given time point
   *
   * Used for online replanning when constraints change during execution
   *
   * @param planning_time Current time point to start replanning from
   * @param gripdata Updated gripper constraints
   * @param traj Trajectory manager to update
   */
  void resolve(double planning_time,
               TrajectoryManager* traj);

 private:
  /**
   * @brief Computes first-order (velocity) constraints
   *
   * Constraints have form: A*x[k] < b
   * Where x[k] is squared velocity (s_dot^2) at waypoint k
   *
   * Includes:
   * - Joint velocity limits: |q_dot| <= v_max
   * - End-effector linear velocity limits (optional)
   *
   * @param k Waypoint index
   * @param constraints Output constraint matrices A, b
   */
  void get1stlimit(int k, InequalData& constraints);

  /**
   * @brief Computes second-order (acceleration/torque) constraints
   *
   * Constraints have form: A*[x[k]; x[k+1]] < b
   * Where x is squared velocity (s_dot^2)
   *
   * Includes:
   * - Joint torque limits: |tau| <= tau_max
   * - Joint acceleration limits: |q_ddot| <= a_max
   * - End-effector linear acceleration limits (optional)
   * - Grasping force limits (optional)
   *
   * Uses finite differencing to approximate acceleration
   *
   * @param k Waypoint index
   * @param constraints Output constraint matrices A, b
   */
  void get2ndlimit(int k, InequalData& constraints);

  /**
   * @brief Computes third-order (jerk) constraints
   *
   * Constraints have form: A*[x[k]; x[k+1]; x[k+2]] < b
   *
   * Includes:
   * - Joint jerk limits: |q_dddot| <= j_max
   * - Continuity constraints on acceleration
   *
   * Uses finite differencing to approximate jerk
   *
   * @param k Waypoint index
   * @param x0_list Current velocity profile
   * @param constraints Output constraint matrices A, b
   */
  void get3rdlimit(int k, const std::vector<double>& x0_list,
                   InequalData& constraints);

  double getFeasibleX(int k);
  double getControllableX(int k, double xmax_c_kk);
  double getReachableXMax(int k, double xmax_c_k, double xmax_r_kpre);

  void initializeSets();
  void initializeDimensions();
  void updateGripdataToSysdata();

  void solveTOPPRA0();
  /**
   * @brief Solves TOPPRA (Time-Optimal Path Parameterization with Reachability Analysis)
   *
   * Implements a two-pass algorithm:
   * 1. Backward pass: Computes maximum controllable velocities
   * 2. Forward pass: Computes maximum reachable velocities
   *
   * Only considers up to 2nd order constraints (velocity, acceleration, torque)
   *
   * @param idx_curr Starting waypoint index for partial replanning (default: 0)
   * @return true if feasible solution found
   */
  bool solveTOPPRA(int idx_curr = 0);
  void solveTOPP3(const std::vector<double>& x0_list, int idx_curr = 0);
  /**
   * @brief Builds complete constraint set for TOPP3 optimization
   *
   * Combines:
   * - First order constraints (velocity)
   * - Second order constraints (acceleration/torque)
   * - Third order constraints (jerk)
   * - Trust region constraints
   *
   * @param x0_list Current velocity profile
   * @param k Starting waypoint index
   * @param h Number of waypoints to optimize
   * @param alpha Trust region size
   * @return List of constraint matrices for each order
   */
  InequalDataList buildTOPP3Ineq(const std::vector<double>& x0_list, int k,
                                 int h, double alpha = 1.);
  /**
   * @brief Updates third order constraints for TOPP3
   *
   * Called during TOPP3 iterations to update constraints based on:
   * - Current velocity profile
   * - Trust region size
   *
   * @param x0_list Current velocity profile
   * @param k Starting waypoint index
   * @param h Number of waypoints to optimize
   * @param TOPP3Ineq Constraint matrices to update
   * @param alpha Trust region size
   */
  void updateTOPP3Ineq(const std::vector<double>& x0_list, int k, int h,
                       InequalDataList& TOPP3Ineq, double alpha = 1.);
  Eigen::VectorXd getCostCoeffs(const std::vector<double>& x0_list, int k,
                                int h);

  // check functions
  double checkLimits(const std::vector<double>& x0_list);
  void checkForcesAtSuction(const std::vector<double>& x0_list);
  Eigen::VectorXd computeJerk(int k, const std::vector<double>& x0_list);
  double estimateMotionPeriod(const std::vector<double>& x0_list);

 protected:
  bool grasping_activated_;
  bool lin_vel_check_activated_;
  bool lin_acc_check_activated_;
  int dim_;
  int dimIneq1_;
  int dimIneq2_;
  int dimIneq3_;
  const double MIN_SDOT_ = 1e-5;
  std::vector<double> feasible_xmax_;
  std::vector<double> controllable_xmax_;
  std::vector<double> reachable_xmax_;
  std::vector<double> trackable_xmax_;

  SYSTEM_DATA sysdata_;
  LPSolver* lpsolver_;
  Clock* clock_;
};
