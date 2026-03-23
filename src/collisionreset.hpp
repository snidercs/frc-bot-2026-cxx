#pragma once

#include <cmath>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>

#include "drivetrain.hpp"
#include "vision.hpp"

namespace indy {

/** Detects robot collisions or obstruction events via Pigeon 2 IMU jerk and
    recovers odometry using a two-tier response:

    **Tier 1 — Gate relaxation (always on jerk confirmation):**
    Calls `vision.setParameters()` with a loose residual window so the Kalman
    filter can self-correct over the next `kCooldownCycles` cycles without any
    discontinuous pose jump.  This is the preferred recovery path.

    **Tier 2 — Hard reset (only when divergence is catastrophic):**
    If a vision measurement disagrees with odometry by more than
    `kHardResetThreshold` while the cooldown is active, `ResetPose()` is called
    and `Parameters::kDefault` is restored immediately.  This covers cases where
    the robot was physically displaced far enough that gradual correction would
    take too long (e.g. pushed across the field).

    When the cooldown expires without a hard reset, `Parameters::kDefault` is
    restored so normal gating resumes cleanly.

    ## Safety properties
    - **IMU-debounced**: requires `kConfirmCycles` consecutive over-threshold
      reads before any action is taken, rejecting single-sample bumps.
    - **Vision-confirmed hard reset**: `ResetPose()` is only issued when *both*
      an IMU spike AND a large pose divergence are seen concurrently.
    - **Cooldown-gated**: minimum `kCooldownCycles` must pass before a new
      event can be armed, preventing repeated resets from one collision.
    - **Teleop/Test only**: callers guard invocation with `IsTeleop() || IsTest()`.
    - **No autonomous interference**: never called during PathPlanner sequences.

    ## Usage
    @code
        // member: indy::CollisionReset _collisionReset;
        // in RobotPeriodic (inside BOT_VISION + teleop guard,
        //   AFTER vision.process() and the fuse loop):
        _collisionReset.update(drive, vision, vision.measurements(), currentPose);
    @endcode
*/
class CollisionReset {
public:
    CollisionReset() = default;

    /** Process one robot-loop cycle.

        Must be called **after** `vision.process()` so that `measurements`
        already contains the current cycle's data.

        @param drivetrain   Supplies the Pigeon 2 and receives `ResetPose()`
                            if a hard reset is warranted.
        @param vision       The active `vision::Processor` — its parameters are
                            loosened on jerk confirmation and restored on expiry.
        @param measurements The vision measurement buffer from this cycle
                            (`vision.measurements()`).
        @param currentPose  The drivetrain's fused pose as sampled **before**
                            `vision.process()` this cycle.
    */
    void update(CommandSwerveDrivetrain& drivetrain,
                vision::Processor& vision,
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

        // ── 3. On a fresh jerk event: Tier 1 — loosen vision gates ──
        if (jerkConfirmed && _cooldown == 0) {
            _cooldown = kCooldownCycles;

            // Build a loose parameter set from current defaults — only widen
            // the residual limits, leave everything else competition-safe.
            auto looseParams = vision::Processor::Parameters::kDefault;
            looseParams.maxResidual   = kLooseResidualOverride;
            looseParams.looseResidual = kLooseResidualOverride;
            vision.setParameters(looseParams);

#if BOT_TRACE_VISION
            frc::SmartDashboard::PutBoolean("Vision/CollisionGateLoose", true);
#endif
        }

        // ── 4. While cooldown is active: check for Tier 2 — catastrophic divergence ──
        if (_cooldown > 0) {
            --_cooldown;

            for (const auto& m : measurements) {
                const units::meter_t divergence =
                    m.pose.Translation().Distance(currentPose.Translation());

                if (divergence > kHardResetThreshold) {
                    // Tier 2: pose is too far gone for gradual correction.
                    drivetrain.ResetPose(m.pose);
                    vision.setParameters(vision::Processor::Parameters::kDefault);
                    _cooldown  = 0;
                    _jerkCount = 0;
                    ++_hardResetCount;

#if BOT_TRACE_VISION
                    frc::SmartDashboard::PutNumber("Vision/CollisionHardResets", _hardResetCount);
                    frc::SmartDashboard::PutNumber("Vision/CollisionJerk_g", magnitude);
                    frc::SmartDashboard::PutNumber("Vision/CollisionDivergence_m", divergence.value());
                    frc::SmartDashboard::PutBoolean("Vision/CollisionGateLoose", false);
#endif
                    return;
                }
            }

            // Cooldown just expired without a hard reset — restore normal gates.
            if (_cooldown == 0) {
                vision.setParameters(vision::Processor::Parameters::kDefault);
#if BOT_TRACE_VISION
                frc::SmartDashboard::PutBoolean("Vision/CollisionGateLoose", false);
#endif
            }
        }

#if BOT_TRACE_VISION
        frc::SmartDashboard::PutNumber("Vision/CollisionJerk_g", magnitude);
        frc::SmartDashboard::PutNumber("Vision/CollisionHardResets", _hardResetCount);
#endif
    }

    /** Returns the total number of hard `ResetPose()` calls since construction. */
    int hardResetCount() const noexcept { return _hardResetCount; }

private:
    // ── Tuning constants ──────────────────────────────────────────────────────

    /** Lateral acceleration threshold that indicates a collision or obstruction.
        Pigeon 2 reports in standard-gravity units (g). X/Y should be ~0 at rest.
        FRC collisions typically register 2–5 g laterally.
        Start conservative and tune: raise if normal driving false-triggers,
        lower if real hits are missed. */
    static constexpr double kJerkThreshold = 1.5; // standard g

    /** Consecutive over-threshold cycles required to confirm a jerk event.
        At 50 Hz: 2 cycles = 40 ms. Rejects single-sample noise from bumps. */
    static constexpr int kConfirmCycles = 2;

    /** Cycles the loose gate and hard-reset window stay open after a jerk event.
        At 50 Hz: 50 cycles ≈ 1 second. */
    static constexpr int kCooldownCycles = 50;

    /** Residual limit applied to both `maxResidual` and `looseResidual` during
        the Tier 1 loose-gate window. Wide enough to let the Kalman filter see
        measurements that a hard collision displaced beyond the normal gate. */
    static constexpr units::meter_t kLooseResidualOverride = 1.5_m;

    /** Divergence above which Tier 2 fires: gradual correction is too slow and
        a hard `ResetPose()` is warranted. Must be > `kLooseResidualOverride`
        so there is always a window where Tier 1 can correct without a hard reset. */
    static constexpr units::meter_t kHardResetThreshold = 1.0_m;

    // ── State ─────────────────────────────────────────────────────────────────
    int _jerkCount     { 0 };
    int _cooldown      { 0 };
    int _hardResetCount{ 0 };
};

} // namespace indy

