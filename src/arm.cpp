#include "arm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm {

namespace {

// Slack for floating point round-off, so a target exactly on the edge of the
// workspace (for example the result of forward()) still counts as reachable.
constexpr double kEpsilon = 1e-9;

bool is_valid_length(double length) {
    return std::isfinite(length) && length > 0.0;
}

// Wraps an angle into the range [-pi, pi].
double normalize_angle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

void check_limit(const JointLimit& limit) {
    if (limit.min > limit.max) {
        throw std::invalid_argument("joint limit min must not be greater than max");
    }
}

bool is_in_range(double angle, const JointLimit& limit) {
    return angle >= limit.min && angle <= limit.max;
}

}  // namespace

PlanarArm::PlanarArm(double l1, double l2, JointLimit shoulder_limit, JointLimit elbow_limit)
    : l1_(l1), l2_(l2) {
    if (!is_valid_length(l1) || !is_valid_length(l2)) {
        throw std::invalid_argument("link lengths must be positive and finite");
    }
    set_joint_limits(shoulder_limit, elbow_limit);
}

void PlanarArm::set_joint_limits(JointLimit shoulder_limit, JointLimit elbow_limit) {
    // Validate both before changing anything, so a bad call leaves the arm untouched.
    check_limit(shoulder_limit);
    check_limit(elbow_limit);
    shoulder_limit_ = shoulder_limit;
    elbow_limit_ = elbow_limit;
}

bool PlanarArm::within_limits(double theta1, double theta2) const {
    return is_in_range(theta1, shoulder_limit_) && is_in_range(theta2, elbow_limit_);
}

Point PlanarArm::forward(double theta1, double theta2) const {
    if (!within_limits(theta1, theta2)) {
        throw std::out_of_range("joint angles are outside the configured limits");
    }
    const double x = l1_ * std::cos(theta1) + l2_ * std::cos(theta1 + theta2);
    const double y = l1_ * std::sin(theta1) + l2_ * std::sin(theta1 + theta2);
    return {x, y};
}

std::optional<JointAngles> PlanarArm::inverse(double x, double y, bool elbow_up) const {
    // Law of cosines: the distance to the target fixes the elbow angle.
    const double distance_squared = x * x + y * y;
    double cos_theta2 = (distance_squared - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);

    // Outside [-1, 1] means the target is too far away or too close to the base.
    // NaN shows up if x or y was NaN.
    if (std::isnan(cos_theta2) || cos_theta2 > 1.0 + kEpsilon || cos_theta2 < -1.0 - kEpsilon) {
        return std::nullopt;
    }
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);

    // Two mirror-image solutions: a negative elbow angle is "elbow up".
    const double sin_magnitude = std::sqrt(1.0 - cos_theta2 * cos_theta2);
    const double sin_theta2 = elbow_up ? -sin_magnitude : sin_magnitude;
    const double theta2 = std::atan2(sin_theta2, cos_theta2);

    // Shoulder angle = direction to the target minus the angle the second link adds.
    const double theta1 =
        normalize_angle(std::atan2(y, x) - std::atan2(l2_ * sin_theta2, l1_ + l2_ * cos_theta2));

    if (!within_limits(theta1, theta2)) {
        return std::nullopt;
    }
    return JointAngles{theta1, theta2};
}

}  // namespace arm
