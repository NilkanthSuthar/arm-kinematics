#include "arm.hpp"

#include <cmath>
#include <stdexcept>

namespace arm {

namespace {

bool is_valid_length(double length) {
    return std::isfinite(length) && length > 0.0;
}

}  // namespace

PlanarArm::PlanarArm(double l1, double l2) : l1_(l1), l2_(l2) {
    if (!is_valid_length(l1) || !is_valid_length(l2)) {
        throw std::invalid_argument("link lengths must be positive and finite");
    }
}

Point PlanarArm::forward(double theta1, double theta2) const {
    const double x = l1_ * std::cos(theta1) + l2_ * std::cos(theta1 + theta2);
    const double y = l1_ * std::sin(theta1) + l2_ * std::sin(theta1 + theta2);
    return {x, y};
}

}  // namespace arm
