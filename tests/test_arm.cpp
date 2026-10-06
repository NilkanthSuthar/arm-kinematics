#include "arm.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using arm::JointLimit;
using arm::kPi;
using arm::PlanarArm;
using arm::Point;

constexpr double kTolerance = 1e-9;

void expect_point_near(const Point& actual, const Point& expected) {
    EXPECT_NEAR(actual.x, expected.x, kTolerance);
    EXPECT_NEAR(actual.y, expected.y, kTolerance);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(PlanarArmConstruction, AcceptsPositiveLengths) {
    EXPECT_NO_THROW(PlanarArm(1.0, 0.5));
}

TEST(PlanarArmConstruction, RejectsZeroOrNegativeLengths) {
    EXPECT_THROW(PlanarArm(0.0, 1.0), std::invalid_argument);
    EXPECT_THROW(PlanarArm(1.0, 0.0), std::invalid_argument);
    EXPECT_THROW(PlanarArm(-1.0, 1.0), std::invalid_argument);
}

TEST(PlanarArmConstruction, RejectsNonFiniteLengths) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_THROW(PlanarArm(nan, 1.0), std::invalid_argument);
    EXPECT_THROW(PlanarArm(1.0, inf), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

TEST(Forward, ZeroAnglesPointAlongXAxis) {
    const PlanarArm arm(2.0, 1.0);
    expect_point_near(arm.forward(0.0, 0.0), {3.0, 0.0});
}

TEST(Forward, ShoulderAtNinetyDegreesPointsUp) {
    const PlanarArm arm(2.0, 1.0);
    expect_point_near(arm.forward(kPi / 2.0, 0.0), {0.0, 3.0});
}

TEST(Forward, ElbowAtNinetyDegrees) {
    const PlanarArm arm(2.0, 1.0);
    expect_point_near(arm.forward(0.0, kPi / 2.0), {2.0, 1.0});
}

TEST(Forward, FoldedBackOnItself) {
    const PlanarArm arm(2.0, 1.0);
    expect_point_near(arm.forward(0.0, kPi), {1.0, 0.0});
}

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

TEST(Inverse, RoundTripRecoversOriginalAngles) {
    const PlanarArm arm(1.5, 1.0);
    const std::array<double, 6> angles = {-2.5, -1.2, -0.3, 0.4, 1.1, 2.8};

    for (const double theta1 : angles) {
        for (const double theta2 : angles) {
            const Point tip = arm.forward(theta1, theta2);
            // A negative elbow angle is the elbow-up solution.
            const auto solution = arm.inverse(tip.x, tip.y, theta2 < 0.0);

            ASSERT_TRUE(solution.has_value()) << "theta1=" << theta1 << " theta2=" << theta2;
            EXPECT_NEAR(solution->theta1, theta1, 1e-6);
            EXPECT_NEAR(solution->theta2, theta2, 1e-6);
        }
    }
}

TEST(Inverse, BothConfigurationsReachTheTarget) {
    const PlanarArm arm(2.0, 1.0);
    const Point target{1.5, 1.2};

    for (const bool elbow_up : {true, false}) {
        const auto solution = arm.inverse(target.x, target.y, elbow_up);
        ASSERT_TRUE(solution.has_value());
        expect_point_near(arm.forward(solution->theta1, solution->theta2), target);
    }
}

TEST(Inverse, ElbowUpAndElbowDownAreMirrorImages) {
    const PlanarArm arm(1.0, 1.0);

    const auto up = arm.inverse(1.0, 0.0, true);
    const auto down = arm.inverse(1.0, 0.0, false);
    ASSERT_TRUE(up.has_value());
    ASSERT_TRUE(down.has_value());

    // Target on the +x axis: elbow up has a negative elbow angle and the elbow above the axis.
    EXPECT_LT(up->theta2, 0.0);
    EXPECT_GT(down->theta2, 0.0);
    EXPECT_GT(std::sin(up->theta1), 0.0);
    EXPECT_LT(std::sin(down->theta1), 0.0);

    // The two solutions mirror each other across the x axis.
    EXPECT_NEAR(up->theta1, -down->theta1, kTolerance);
    EXPECT_NEAR(up->theta2, -down->theta2, kTolerance);
}

TEST(Inverse, TargetBeyondReachIsUnreachable) {
    const PlanarArm arm(2.0, 1.0);
    EXPECT_FALSE(arm.inverse(3.5, 0.0, true).has_value());
    EXPECT_FALSE(arm.inverse(0.0, -3.01, false).has_value());
}

TEST(Inverse, TargetInsideInnerHoleIsUnreachable) {
    // With l1 = 2 and l2 = 1 the arm cannot get closer than 1 to the base.
    const PlanarArm arm(2.0, 1.0);
    EXPECT_FALSE(arm.inverse(0.5, 0.0, true).has_value());
    EXPECT_FALSE(arm.inverse(0.0, 0.0, false).has_value());
}

TEST(Inverse, NanTargetIsUnreachable) {
    const PlanarArm arm(2.0, 1.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(arm.inverse(nan, 1.0, true).has_value());
}

TEST(Inverse, SingularityAtFullExtension) {
    const PlanarArm arm(2.0, 1.0);
    const double reach = 3.0;
    const double direction = 0.7;  // any direction works
    const double x = reach * std::cos(direction);
    const double y = reach * std::sin(direction);

    const auto up = arm.inverse(x, y, true);
    const auto down = arm.inverse(x, y, false);
    ASSERT_TRUE(up.has_value());
    ASSERT_TRUE(down.has_value());

    // Arm is straight: elbow angle is zero and both configurations collapse into one.
    EXPECT_NEAR(up->theta2, 0.0, kTolerance);
    EXPECT_NEAR(up->theta1, direction, kTolerance);
    EXPECT_NEAR(up->theta1, down->theta1, kTolerance);
    EXPECT_NEAR(up->theta2, down->theta2, kTolerance);

    // A hair further out is no longer reachable.
    EXPECT_FALSE(arm.inverse(x * 1.0001, y * 1.0001, true).has_value());
}

TEST(Inverse, FullyFoldedArmIsReachable) {
    // Inner edge of the workspace: elbow folded back completely.
    const PlanarArm arm(2.0, 1.0);
    const auto solution = arm.inverse(1.0, 0.0, true);
    ASSERT_TRUE(solution.has_value());
    EXPECT_NEAR(std::abs(solution->theta2), kPi, kTolerance);
    expect_point_near(arm.forward(solution->theta1, solution->theta2), {1.0, 0.0});
}

// ---------------------------------------------------------------------------
// Joint limits
// ---------------------------------------------------------------------------

TEST(JointLimits, DefaultLimitsAreMinusPiToPi) {
    const PlanarArm arm(1.0, 1.0);
    EXPECT_TRUE(arm.within_limits(kPi, -kPi));
    EXPECT_FALSE(arm.within_limits(kPi + 0.01, 0.0));
    EXPECT_FALSE(arm.within_limits(0.0, -kPi - 0.01));
}

TEST(JointLimits, ConstructorRejectsInvertedLimit) {
    EXPECT_THROW(PlanarArm(1.0, 1.0, JointLimit{1.0, -1.0}), std::invalid_argument);
    EXPECT_THROW(PlanarArm(1.0, 1.0, JointLimit{}, JointLimit{0.5, 0.0}), std::invalid_argument);
}

TEST(JointLimits, SetterReplacesLimits) {
    PlanarArm arm(1.0, 1.0);
    arm.set_joint_limits({-0.5, 0.5}, {0.0, 1.0});
    EXPECT_TRUE(arm.within_limits(0.5, 0.0));
    EXPECT_FALSE(arm.within_limits(0.6, 0.0));
    EXPECT_FALSE(arm.within_limits(0.0, -0.1));
}

TEST(JointLimits, SetterRejectsInvertedLimitAndKeepsOldOnes) {
    PlanarArm arm(1.0, 1.0, {-0.5, 0.5}, {-0.5, 0.5});
    EXPECT_THROW(arm.set_joint_limits({-1.0, 1.0}, {1.0, -1.0}), std::invalid_argument);
    // The shoulder limit must not have been widened to [-1, 1].
    EXPECT_FALSE(arm.within_limits(0.8, 0.0));
}

TEST(JointLimits, ForwardRejectsAnglesOutsideLimits) {
    const PlanarArm arm(1.0, 1.0, {-1.0, 1.0}, {0.0, 2.0});
    EXPECT_NO_THROW((void)arm.forward(1.0, 2.0));
    EXPECT_THROW((void)arm.forward(1.5, 1.0), std::out_of_range);
    EXPECT_THROW((void)arm.forward(0.0, -0.5), std::out_of_range);
}

TEST(JointLimits, InverseRejectsSolutionOutsideElbowLimit) {
    // The elbow can only bend one way, so only the elbow-down (theta2 >= 0) solution exists.
    const PlanarArm arm(1.0, 1.0, JointLimit{}, JointLimit{0.0, kPi});
    EXPECT_FALSE(arm.inverse(1.0, 1.0, true).has_value());
    EXPECT_TRUE(arm.inverse(1.0, 1.0, false).has_value());
}

TEST(JointLimits, InverseRejectsSolutionOutsideShoulderLimit) {
    // Shoulder restricted to the upper half plane: a target straight below is out.
    const PlanarArm arm(1.0, 1.0, JointLimit{0.0, kPi}, JointLimit{});
    EXPECT_FALSE(arm.inverse(0.0, -1.5, true).has_value());
    EXPECT_FALSE(arm.inverse(0.0, -1.5, false).has_value());
    EXPECT_TRUE(arm.inverse(0.0, 1.5, true).has_value());
}

}  // namespace
