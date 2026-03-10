#pragma once

#include "drivetrain.hpp"
#include "vision.hpp"

namespace indy {

/** One-shot vision pose reset utility.
 
    On the first call to `tryReset()` that finds a valid vision measurement,
    resets the drivetrain pose to that measurement and latches — subsequent
    calls are no-ops.  Construct a fresh instance (or call `reset()`) whenever
    you want to arm it for a new attempt (e.g. on TeleopInit).

    Intentionally not a Command so it can be used freely from Robot periodic
    methods, autonomous routines, or tests without scheduler overhead.

    Typical teleop usage:
    @code
        // member: indy::PoseResetOnce _poseReset;
        // in TeleopInit:  _poseReset.reset();
        // in RobotPeriodic (teleop guard): _poseReset.tryReset(vision, drivetrain);
    @endcode
*/
class PoseResetOnce {
public:
    PoseResetOnce() = default;

    /** Re-arm so the next call to tryReset() will attempt the reset again. */
    void reset() noexcept { _done = false; }

    /** Returns true once a successful reset has been performed. */
    bool isDone() const noexcept { return _done; }

    /** Attempt a pose reset from the first valid vision measurement.
     
        If already done, returns immediately.  Otherwise iterates the current
        measurement buffer from @p vision and, on the first entry found, calls
        `drivetrain.ResetPose()` and latches.

        @param vision     The active VisionIO source.
        @param drivetrain The swerve drivetrain whose pose will be reset.
        @return true if a reset was performed on this call, false otherwise.
    */
    bool tryReset(VisionIO& vision, CommandSwerveDrivetrain& drivetrain)
    {
        if (_done)
            return false;

        for (const auto& m : vision.getMeasurements()) {
            drivetrain.ResetPose(m.pose);
            _done = true;
            return true;
        }

        return false;
    }

private:
    bool _done { false };
};

} // namespace indy
