#include <gtest/gtest.h>
#include "inpututil.hpp"

// Shared test parameters
static constexpr double kDeadband = 0.1;
static constexpr double kExponent = 2.0;
static constexpr double kTolerance = 1e-9;

// ---------------------------------------------------------------------------
// Dead zone: anything at or below the deadband threshold must return exactly 0
// ---------------------------------------------------------------------------

TEST(ApplyCurveTest, ZeroInputReturnsZero) {
    EXPECT_DOUBLE_EQ(bot::applyCurve(0.0, kDeadband, kExponent), 0.0);
}

TEST(ApplyCurveTest, WithinDeadbandReturnsZero) {
    EXPECT_DOUBLE_EQ(bot::applyCurve(0.05, kDeadband, kExponent), 0.0);
    EXPECT_DOUBLE_EQ(bot::applyCurve(-0.05, kDeadband, kExponent), 0.0);
}

TEST(ApplyCurveTest, AtDeadbandEdgeReturnsZero) {
    // The curve starts AT the deadband — the edge itself must still be 0
    EXPECT_DOUBLE_EQ(bot::applyCurve(kDeadband, kDeadband, kExponent), 0.0);
    EXPECT_DOUBLE_EQ(bot::applyCurve(-kDeadband, kDeadband, kExponent), 0.0);
}

// ---------------------------------------------------------------------------
// Intersection: full deflection (1.0) must always map to 1.0 regardless of
// deadband or exponent — i.e. the curve passes through (1, 1) on the graph
// ---------------------------------------------------------------------------

TEST(ApplyCurveTest, FullDeflectionReturnsOne) {
    EXPECT_NEAR(bot::applyCurve(1.0, kDeadband, kExponent), 1.0, kTolerance);
    EXPECT_NEAR(bot::applyCurve(-1.0, kDeadband, kExponent), -1.0, kTolerance);
}

TEST(ApplyCurveTest, FullDeflectionReturnsOneWithCubicExponent) {
    EXPECT_NEAR(bot::applyCurve(1.0, kDeadband, 3.0), 1.0, kTolerance);
    EXPECT_NEAR(bot::applyCurve(-1.0, kDeadband, 3.0), -1.0, kTolerance);
}

TEST(ApplyCurveTest, FullDeflectionReturnsOneWithLinearExponent) {
    // Exponent of 1.0 should behave like a normalized linear function
    EXPECT_NEAR(bot::applyCurve(1.0, kDeadband, 1.0), 1.0, kTolerance);
    EXPECT_NEAR(bot::applyCurve(-1.0, kDeadband, 1.0), -1.0, kTolerance);
}

// ---------------------------------------------------------------------------
// Symmetry: negative input must mirror positive input
// ---------------------------------------------------------------------------

TEST(ApplyCurveTest, NegativeInputMirrorsPositive) {
    const double pos = bot::applyCurve(0.6, kDeadband, kExponent);
    const double neg = bot::applyCurve(-0.6, kDeadband, kExponent);
    EXPECT_NEAR(pos, -neg, kTolerance);
}

// ---------------------------------------------------------------------------
// Monotonicity: output must increase as input increases past the deadband
// ---------------------------------------------------------------------------

TEST(ApplyCurveTest, OutputIsMonotonicallyIncreasing) {
    double prev = 0.0;
    for (int i = 1; i <= 10; ++i) {
        const double input = kDeadband + (1.0 - kDeadband) * (i / 10.0);
        const double output = bot::applyCurve(input, kDeadband, kExponent);
        EXPECT_GT(output, prev) << "Output not increasing at input=" << input;
        prev = output;
    }
}

// ---------------------------------------------------------------------------
// Curve shape: with exponent > 1 the output midpoint should be below the
// input midpoint (i.e. the curve bows toward the axis, giving finer control
// near the deadband edge)
// ---------------------------------------------------------------------------

TEST(ApplyCurveTest, CurveIsBelow_LinearMidpoint_WithQuadraticExponent) {
    // Midpoint of the active range
    const double mid = kDeadband + (1.0 - kDeadband) * 0.5;
    const double curvedOutput = bot::applyCurve(mid, kDeadband, kExponent);    // quadratic
    const double linearOutput  = bot::applyCurve(mid, kDeadband, 1.0);          // linear

    EXPECT_LT(curvedOutput, linearOutput)
        << "Quadratic curve should produce less output than linear at midpoint";
}
