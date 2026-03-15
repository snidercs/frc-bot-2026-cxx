#pragma once

#include <array>
#include <cmath>
#include <units/length.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

namespace indy::math {

/** Returns true if a pose is outside the field boundary by more than @p margin.

    A pose is considered out of bounds when it is clearly wrong — further than
    @p margin beyond any edge. Use this to gate calls to clampPoseToField() so
    that normal sensor noise near a wall does not trigger a ResetPose().

    @param pose        The pose to test.
    @param fieldLength Field length in metres (X axis). 2026: 16.535_m.
    @param fieldWidth  Field width in metres (Y axis).  2026: 8.069_m.
    @param margin      How far outside the boundary before considered OOB.
    @return true if the pose is outside [−margin, fieldLength+margin] × [−margin, fieldWidth+margin].
*/
[[nodiscard]] inline bool isPoseOutOfBounds(
    const frc::Pose2d& pose,
    units::meter_t fieldLength,
    units::meter_t fieldWidth,
    units::meter_t margin) noexcept
{
    return pose.X() < -margin || pose.X() > fieldLength + margin
        || pose.Y() < -margin || pose.Y() > fieldWidth  + margin;
}

/** Clamps a robot pose to within the physical field boundaries.

    Any pose component outside [0, fieldLength] x [0, fieldWidth] is snapped
    to the nearest in-bounds position. Rotation is preserved unchanged.

    Intended as a last-resort backstop in RobotPeriodic() to prevent odometry
    from drifting through walls when the robot is obstructed. Should only fire
    when the pose is clearly wrong — use a margin when deciding whether to call
    this (e.g. only call when pose is > 0.75m outside the boundary).

    @param pose        The pose to clamp.
    @param fieldLength Field length in metres (X axis). 2026: 16.535_m.
    @param fieldWidth  Field width in metres (Y axis).  2026: 8.069_m.
    @return A new Pose2d with X and Y clamped to [0, fieldLength] x [0, fieldWidth].
*/
[[nodiscard]] inline frc::Pose2d clampPoseToField(
    const frc::Pose2d& pose,
    units::meter_t fieldLength,
    units::meter_t fieldWidth) noexcept
{
    const auto x = units::math::max(0_m, units::math::min(pose.X(), fieldLength));
    const auto y = units::math::max(0_m, units::math::min(pose.Y(), fieldWidth));
    return frc::Pose2d{x, y, pose.Rotation()};
}

/** Computes the mean positional variance of a fixed-size rolling window of 2D
    translations. Returns the average squared distance from the centroid across
    all valid samples.

    Intended for use in adaptive stdDev scaling (Option F) where a high variance
    indicates inconsistent vision measurements that should be trusted less by the
    Kalman filter.

    @tparam N          Capacity of the rolling window array.
    @param  positions  The circular buffer of Translation2d samples.
    @param  count      Number of valid entries in the buffer (may be < N).
    @return Mean variance in m². Returns 0.0 if fewer than 2 samples are present.
*/
template <std::size_t N>
[[nodiscard]] double rollingVariance2d(
    const std::array<frc::Translation2d, N>& positions, int count) noexcept
{
    if (count < 2) return 0.0;

    double sumX{0.0}, sumY{0.0};
    for (int i = 0; i < count; ++i) {
        sumX += positions[i].X().value();
        sumY += positions[i].Y().value();
    }
    const double meanX = sumX / count;
    const double meanY = sumY / count;

    double variance{0.0};
    for (int i = 0; i < count; ++i) {
        const double dx = positions[i].X().value() - meanX;
        const double dy = positions[i].Y().value() - meanY;
        variance += dx * dx + dy * dy;
    }
    return variance / count;
}

} // namespace indy::math
