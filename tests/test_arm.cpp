#include "arm.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

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

}  // namespace
