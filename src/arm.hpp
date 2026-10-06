/**
 * @file arm.hpp
 * @brief Forward and inverse kinematics for a 2-link planar robot arm.
 *
 * The base of the arm sits at the origin. Angles are in radians and measured
 * counter-clockwise: theta1 from the +x axis, theta2 relative to the first link.
 */
#pragma once

#include <optional>

namespace arm {

/// Value of pi used throughout the library.
inline constexpr double kPi = 3.14159265358979323846;

/// A point in the plane of the arm, in the same length unit as the links.
struct Point {
    double x = 0.0;  ///< Horizontal position.
    double y = 0.0;  ///< Vertical position.
};

/// One joint configuration of the arm.
struct JointAngles {
    double theta1 = 0.0;  ///< Shoulder angle in radians, measured from the +x axis.
    double theta2 = 0.0;  ///< Elbow angle in radians, measured from the first link.
};

/// Inclusive range of allowed angles for one joint, in radians.
struct JointLimit {
    double min = -kPi;  ///< Smallest allowed angle.
    double max = kPi;   ///< Largest allowed angle.
};

/**
 * @brief A 2-link planar arm (shoulder joint at the origin, elbow joint, tool tip).
 */
class PlanarArm {
public:
    /**
     * @brief Creates an arm with the given link lengths and joint limits.
     * @param l1 Length of the first link (shoulder to elbow). Must be positive.
     * @param l2 Length of the second link (elbow to tool tip). Must be positive.
     * @param shoulder_limit Allowed range for theta1. Defaults to [-pi, pi].
     * @param elbow_limit Allowed range for theta2. Defaults to [-pi, pi].
     * @throws std::invalid_argument if a length is zero, negative or not finite,
     *         or if a limit has min greater than max.
     */
    PlanarArm(double l1, double l2, JointLimit shoulder_limit = {}, JointLimit elbow_limit = {});

    /**
     * @brief Replaces the joint limits.
     * @param shoulder_limit Allowed range for theta1.
     * @param elbow_limit Allowed range for theta2.
     * @throws std::invalid_argument if a limit has min greater than max.
     *         The old limits are kept in that case.
     */
    void set_joint_limits(JointLimit shoulder_limit, JointLimit elbow_limit);

    /**
     * @brief Checks whether both angles are inside the configured joint limits.
     * @param theta1 Shoulder angle in radians.
     * @param theta2 Elbow angle in radians.
     * @return true if both angles are within their limits (limits are inclusive).
     */
    [[nodiscard]] bool within_limits(double theta1, double theta2) const;

    /**
     * @brief Computes the tool tip position for the given joint angles.
     * @param theta1 Shoulder angle in radians.
     * @param theta2 Elbow angle in radians.
     * @return The (x, y) position of the tool tip.
     * @throws std::out_of_range if an angle is outside its joint limit.
     */
    [[nodiscard]] Point forward(double theta1, double theta2) const;

    /**
     * @brief Finds joint angles that put the tool tip at (x, y).
     *
     * Most reachable points have two solutions. With @p elbow_up the elbow sits on the
     * counter-clockwise side of the line from the base to the target (above it for a
     * target on the +x axis), which means theta2 <= 0. Otherwise theta2 >= 0.
     * At full extension or full fold both solutions are the same.
     *
     * @param x Target x position.
     * @param y Target y position.
     * @param elbow_up Which of the two solutions to return.
     * @return The joint angles, with theta1 in [-pi, pi] and theta2 in [-pi, pi],
     *         or std::nullopt if the target is out of reach or the requested
     *         solution breaks a joint limit.
     */
    [[nodiscard]] std::optional<JointAngles> inverse(double x, double y, bool elbow_up) const;

private:
    double l1_;
    double l2_;
    JointLimit shoulder_limit_;
    JointLimit elbow_limit_;
};

}  // namespace arm
