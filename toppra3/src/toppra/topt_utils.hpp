#ifndef TOPT_UTILS_H
#define TOPT_UTILS_H

#include "toppra/user_command.hpp"
#include "toppra/math/math_utilities.hpp"

namespace topt_utils {
Eigen::MatrixXd hZeroStack(int Nl, const Eigen::MatrixXd& A) {
  // [zero(Nl), A]
  int r = A.rows();
  return toppra::hStack(Eigen::MatrixXd::Zero(r, Nl), A);
}
Eigen::MatrixXd hZeroStack(const Eigen::MatrixXd& A, int Nr) {
  // [A, zero(Nr)]
  int r = A.rows();
  return toppra::hStack(A, Eigen::MatrixXd::Zero(r, Nr));
}
Eigen::MatrixXd hZeroStack(int Nl, const Eigen::MatrixXd& A, int Nr) {
  // [zero(Nl), A, zero(Nr)]
  int r = A.rows();
  Eigen::MatrixXd temp = toppra::hStack(Eigen::MatrixXd::Zero(r, Nl), A);
  return toppra::hStack(temp, Eigen::MatrixXd::Zero(r, Nr));
}

}  // namespace topt_utils

#endif  // TOPT_UTILS_H