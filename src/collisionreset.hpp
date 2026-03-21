#pragma once

#include <cmath>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/acceleration.h>
#include <units/length.h>

#include "drivetrain.hpp"
#include "vision.hpp"

namespace indy {

/** Detects robot collisions or obstruction events via Pigeon 2 IMU jerk and
    performs a hard odometry reset from the best available vision measurement.

    ## How it works

    Every call to `update()`:
    1. Reads the lateral (X/Y) acceleration magnitude from the Pigeon 2.
    2. If that magnitude exceeds `kJerkThreshold` for at least `kConfirmCycles`
       consecutive cycles, a **jerk event** is latched and a cooldown window of
       `kCooldownCycles` opens.
    3. While the cooldown is active, the first vision measurement that disagrees
       with the current fused odometry pose by more than `kDivergenceThreshold`
       triggers a `ResetPose()` to that vision pose.  If vision agrees (or is
       absent), no reset is issued — the odometry is still good.
    4. The cooldown expires after `kCooldownCycles` whether or not a reset
       fires, preventing repeated resets from a single event.

    ## Safety properties
    - **Vision-confirmed**: a reset is only issued when *both* an IMU spike
      AND a measurable pose divergence are seen.  A hard bump that doesn't
      move the robot (e.g. another robot bouncing off) will not reset anything.
    - **Cooldown-gated**: a minimum of `kCooldownCycles` robot-loop cycles
      must pass before another reset can be armed (≈ 1 s at 50 Hz).
    - **Teleop/Test only**: the caller guards invocation with `IsTeleop() || IsTest()`.
    - **No autonomous interference**: never called during PathPlanner sequences.

    ## Usage
    @code
        // in Robot member:  indy::CollisionReset _collisionReset;
        // in RobotPeriodic (inside the BOT_VISION + teleop guard,
        //   AFTER vision.process() and the fuse loop):
        _collisionReset.update(drive, vision.measurements(), currentPose);
    @endcode
*/
class CollisionReset {
public:
    CollisionReset() = default;

    /** Process one robot-loop cycle.

        Must be called **after** `vision.process()` so that
        `measurements` already contains the current cycle's data.

        @param drivetrain   The swerve drivetrain — supplies the Pigeon 2 and
                            receives `ResetPose()` when triggered.
        @param measurements The vision measurement buffer from this cycle
                            (i.e. the result of `vision.measurements()`).
        @param currentPose  The drivetrain's fused pose **as sampled before**
                            `vision.process()` this cycle (same value passed
                            to `vision.process()`).
    */
    void update(CommandSwerveDrivetrain& drivetrain,
                const std::vector<vision::Measurement>& measurements,
                const frc::Pose2d& currentPose)
    {
        // ── 1. Read Pigeon 2 lateral acceleration (X and Y in robot frame) ──
        auto& pigeon = drivetrain.GetPigeon2();
        const double ax = pigeon.GetAccelerationX(true).GetValue().value();
        const double ay = pigeon.GetAccelerationY(true).GetValue().value();
        const double magnitude = std::sqrt(ax * ax + ay * ay);

        // ── 2. Debounce: require kConfirmCycles consecutive over-threshold reads ──
        if (magnitude > kJerkThreshold) {
            if (_jerkCount < kConfirmCycles)
                ++_jerkCount;
        } else {
            _jerkCount = 0;
        }

        const bool jerkConfirmed = (_jerkCount >= kConfirmCycles);

        // ── 3. Arm the cooldown window on a fresh jerk event ──
        if (jerkConfirmed && _cooldown == 0) {
            _cooldown = kCooldownCycles;
        }

        // ── 4. While cooldown is active, look for vision divergence ──
        if (_cooldown > 0) {
            --_cooldown;

            for (const auto& m : measurements) {
                const units::meter_t divergence =
                    m.pose.Translation().Distance(currentPose.Translation());

                if (divergence > kDivergenceThreshold) {
                    drivetrain.ResetPose(m.pose);
                    // Suppress remaining cooldown — one reset per event is enough.
                    _cooldown = 0;
                    _jerkCount = 0;
                    ++_resetCount;

#if BOT_TRACE_VISION
                    frc::SmartDashboard::PutNumber(
                        "Vision/CollisionResets", _resetCount);
                    frc::SmartDashboard::PutNumber(
                        "Vision/CollisionJerk_g", magnitude);
                    frc::SmartDashboard::PutNumber(
                        "Vision/CollisionDivergence_m", divergence.value());
#endif
                    return;
                }
            }
        }

#if BOT_TRACE_VISION
        frc::SmartDashboard::PutNumber("Vision/CollisionJerk_g", magnitude);
        frc::SmartDashboard::PutNumber("Vision/CollisionResets", _resetCount);
#endif
    }

    /** Returns the total number of hard resets performed since construction. */
    int resetCount() const noexcept { return _resetCount; }

private:
    // ── Tuning constants ──────────────────────────────────────────────────────

    /** Lateral acceleration threshold that indicates a collision or obstruction.
        The Pigeon 2 reports in standard-gravity units (1 g = 9.81 m/s²).
        At rest the Z axis reads ~1 g; X/Y should be near 0.
        FRC collisions typically register 2–5 g laterally.
        Start conservative (1.5 g) and tune down if events are missed,
        or up if normal driving triggers false positives. */
    static constexpr double kJerkThreshold = 1.5; // standard g

    /** Number of consecutive over-threshold cycles required before a jerk
        event is confirmed.  At 50 Hz one cycle = 20 ms; 2 cycles = 40 ms.
        This rejects single-sample noise spikes from driving over a bump. */
    static constexpr int kConfirmCycles = 2;

    /** How many robot-loop cycles the reset window stays open after a
        confirmed jerk event.  At 50 Hz: 50 cycles ≈ 1 second.
        Vision must produce a diverging measurement within this window. */
    static constexpr int kCooldownCycles = 50;

    /** Minimum pose disagreement (vision vs odometry) required to commit a
        reset.  Below this the odometry is still trustworthy and no reset
        is needed even after a bump. */
    static constexpr units::meter_t kDivergenceThreshold = 0.3_m;

    // ── State ─────────────────────────────────────────────────────────────────
    int _jerkCount  { 0 };
    int _cooldown   { 0 };
    int _resetCount { 0 };
};

} // namespace indy
