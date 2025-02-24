#include "math/linear_interpolator.hpp"

namespace toppra {
namespace math {

LinearInterpolator::LinearInterpolator(const std::vector<double>& x,
                                       const std::vector<double>& y) {
  if (x.size() != y.size()) {
    throw std::invalid_argument(
        "x and y vectors must have the same size got x.size() = " +
        std::to_string(x.size()) + " and y.size() = " + std::to_string(y.size()));
  }
  if (x.size() < 2) {
    throw std::invalid_argument("Need at least 2 points for interpolation");
  }

  // Verify x is strictly increasing
  for (size_t i = 1; i < x.size(); ++i) {
    if (x[i] <= x[i - 1]) {
      throw std::invalid_argument(
          "x values must be strictly increasing got x[i] = " + std::to_string(x[i]) +
          " and x[i-1] = " + std::to_string(x[i - 1]));
    }
  }

  x_values_ = x;
  y_values_ = y;
}

double LinearInterpolator::interpolate(double x) const {
  if (x < x_values_.front() || x > x_values_.back() + 1e-5) {
    throw std::out_of_range(
        "x value outside interpolation range x = " + std::to_string(x) +
        " x_values_.front() = " + std::to_string(x_values_.front()) +
        " x_values_.back() = " + std::to_string(x_values_.back()));
  }

  if (x > x_values_.back()) {
    return y_values_.back();
  }

  // Find the right interval using binary search
  auto it = std::lower_bound(x_values_.begin(), x_values_.end(), x);

  if (it == x_values_.begin()) {
    return y_values_.front();
  }

  size_t right_idx = std::distance(x_values_.begin(), it);
  size_t left_idx = right_idx - 1;

  // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
  double x1 = x_values_[left_idx];
  double x2 = x_values_[right_idx];
  double y1 = y_values_[left_idx];
  double y2 = y_values_[right_idx];

  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

}  // namespace math
}  // namespace toppra
