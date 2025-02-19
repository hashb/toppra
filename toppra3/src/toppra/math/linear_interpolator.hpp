#pragma once

#include <vector>
#include <stdexcept>

namespace toppra {
namespace math {

class LinearInterpolator {
public:
    /**
     * @brief Constructs a linear interpolator from x and y data points
     * @param x Vector of x coordinates (must be strictly increasing)
     * @param y Vector of y coordinates (must be same size as x)
     * @throws std::invalid_argument if inputs are invalid
     */
    LinearInterpolator(const std::vector<double>& x, const std::vector<double>& y);

    /**
     * @brief Interpolates to find y value at given x
     * @param x The x coordinate to interpolate at
     * @return The interpolated y value
     * @throws std::out_of_range if x is outside the interpolation range
     */
    double interpolate(double x) const;

    /**
     * @brief Get the minimum x value in the interpolation range
     */
    double getMinX() const { return x_values_.front(); }

    /**
     * @brief Get the maximum x value in the interpolation range
     */
    double getMaxX() const { return x_values_.back(); }

private:
    std::vector<double> x_values_;
    std::vector<double> y_values_;
};

} // namespace math
} // namespace toppra
