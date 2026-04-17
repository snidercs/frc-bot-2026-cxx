#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <wpi/array.h>

#include "config.hpp"

namespace indy::vision {

/** A single vision pose measurement from a camera.
 
    Contains the estimated robot pose, timestamp, standard deviations for
    pose uncertainty, and the source camera that produced the measurement.
*/
struct Measurement {
    /** The estimated robot pose on the field */
    frc::Pose2d pose;

    /** The timestamp when this measurement was captured */
    units::second_t timestamp;

    /** Standard deviations for pose uncertainty [x, y, theta] */
    wpi::array<double, 3> stdDevs;

    /** Source camera name (FL, FR, BL, BR) */
    std::string source;
};

/** Intermediate candidate built from one `PhotonPipelineResult` during gating.
 
    Surviving candidates are ranked and the single best one is promoted into 
    valid measurements.
*/
struct Candidate {
    /** Fully-formed measurement ready to pass to `AddVisionMeasurement()`. */
    Measurement measurement;

    /** Number of AprilTags used in the PNP pose solve.
        Higher is better — more tags means a more constrained, trusted solve. */
    int tagCount { 0 };

    /** 3-D Euclidean distance in metres from the camera to the best visible tag.
        Computed as √(x²+y²+z²) from `GetBestCameraToTarget()`. Used by
        `computeStdDevs()` to scale trust (closer = tighter stdDevs) and as a
        tiebreaker when two candidates have equal tag counts. */
    double distance { 0.0 };
};

/** Interface for vision measurement sources (real hardware or simulation).
 
    Provides a hardware abstraction layer for vision systems, allowing the
    same code to work with both PhotonVision cameras and simulation.

    ## Extending Processor
    Common pipeline logic lives here so subclasses stay thin. The only thing
    a subclass must supply is *how to get raw results* for each camera — call
    `processResults()` with those results and everything else (gating, std-dev
    scaling, SmartDashboard publishing, rejection counting) is handled here.

    To add new shared behaviour, add it here rather than duplicating it in
    both `VisionMulti` and `VisionSim`.
*/
class Processor {
public:
    Processor();
    virtual ~Processor() = default;

    /** Returns the measurement buffer populated by the last call to `process()`.
     
        Does **not** poll cameras or mutate any state — safe to call multiple
        times per cycle. The buffer is valid until the next `process()` call.

        @return Const reference to the internal measurement buffer.
    */
    const std::vector<Measurement>& measurements() const noexcept { return _measurements; }

    /** Polls all cameras for this cycle and fills the measurement buffer.
     
        @warning Call **exactly once per periodic cycle**. Each call clears the
        internal buffer and consumes all unread frames from the PhotonVision NT
        ringbuffer — a second call in the same cycle will return an empty buffer.

        After this returns, iterate results via `measurements()`.

        @param currentPose The drivetrain's current fused pose, already clamped
               to field bounds if needed. Used by `processResults()` for residual
               gating — any candidate further than `kMaxResidual` from this pose
               is rejected before it can reach `AddVisionMeasurement()`.
        @param speeds The drivetrain's current measured chassis speeds (`GetState().Speeds`).
               Stored as `_lastSpeeds` for Option A obstruction detection — when
               commanded motion is high but actual speed is near-zero for several
               consecutive cycles, `computeStdDevs()` can inflate stdDevs to
               increase vision trust during the obstruction window.
    */
    void process (const frc::Pose2d& pose, const frc::ChassisSpeeds& speeds);

    /** Gets a human-readable status string for debugging.
     
        @return Status information about the vision system
    */
    virtual std::string getStatus() { return "Processor base class"; }

    /** Gets the most recent target information from all cameras.
     
        @return String describing detected targets (tag IDs, distances, etc.)
    */
    virtual std::string getLastTargets() { return "No targets"; }

    /** Gets counts of rejected measurements for debugging.
     
        @return String with rejection statistics (stale, ambiguous, etc.)
    */
    virtual std::string getRejectedCounts();

    /** All pipeline gating thresholds in one swappable struct.

        The default-constructed values are the safe competition settings.
        External callers (e.g. `CollisionReset`) may call `setParameters()`
        to temporarily widen the gates and then restore `kDefault` once the
        event window has elapsed.
    */
    struct Parameters {
        /** Maximum accepted pose ambiguity for single-tag solves (0–1 scale).
            PhotonLib reports 0 = perfect, 1 = fully ambiguous. Values below
            0.2 are considered reliable for single-tag use. Multi-tag PNP
            solves bypass this check entirely — they are always accepted if
            they pass the other gates. */
        double maxAmbiguity { 0.2 };

        /** Maximum frame latency before a result is discarded as stale. */
        units::second_t maxLatency { 0.25_s };

        /** Maximum camera-to-tag distance in metres. */
        double maxTagDistance { 5.5 };

        /** Normal odometry residual gate: vision poses further than this
            from the current fused pose are rejected. */
        units::meter_t maxResidual { 1.0_m };

        /** Loose residual gate used when `_dropouts` exceeds
            `dropoutLooseThreshold` or when externally set via `setParameters()`.
            Wide enough to let the Kalman filter self-correct after a collision. */
        units::meter_t looseResidual { 2.0_m };

        /** Number of consecutive dropout cycles before the residual gate
            switches from `maxResidual` to `looseResidual`. */
        uint32_t dropoutLooseThreshold { 3 };

        /** Maximum physically plausible implied robot velocity between two
            accepted vision poses. Solves above this are rejected. */
        units::meters_per_second_t maxImpliedVelocity { 5.0_mps };

        /** Safe competition defaults — always restore to this after any
            temporary gate relaxation. */
        static const Parameters kDefault;
    };

    /** Replace the active pipeline gating parameters.

        Called by external systems (e.g. `CollisionReset`) to widen gates
        temporarily.  Restore with `setParameters(Parameters::kDefault)` once
        the event window expires.

        @param params The new parameter set to apply immediately.
    */
    void setParameters(const Parameters& params) noexcept { _params = params; }

    /** Returns the currently active pipeline parameters. */
    const Parameters& parameters() const noexcept { return _params; }

protected:
    Parameters _params {};

    // Buffers and state tracking
    std::vector<Measurement> _measurements;
    std::vector<Candidate> _candidates;
    frc::Pose2d _lastPose;
    frc::ChassisSpeeds _lastSpeeds;

    // Rejection counters
    int _rejectedNoTargets = 0;
    int _rejectedStale = 0;
    int _rejectedAmbiguous = 0;
    int _rejectedOutOfBounds = 0;
    int _rejectedResidual = 0;
    int _rejectedVelocity = 0;
    int _acceptedCount = 0;

    uint32_t _dropouts = 0;

    // Per-camera velocity gate state
    // Tracks the last committed accepted pose and its timestamp for each camera
    // so the velocity gate can reject solves that imply physically impossible motion.
    struct CameraAnchor {
        frc::Pose2d lastPose {};
        units::second_t lastTime { 0_s };

#if BOT_ADAPTIVE_STDDEVS
        // Rolling window for adaptive stdDev scaling.
        static constexpr int kWindowSize = 6;
        std::array<frc::Translation2d, kWindowSize> recentPositions {};
        int windowCount { 0 };
        int windowHead { 0 };
#endif
    };
    std::unordered_map<std::string, CameraAnchor> _cameraState;

    /** Scales pose std devs by distance and tag count: closer + more tags = more trusted.
     
        @param distanceMeters Distance from camera to nearest tag in metres.
        @param tagCount       Number of tags used in the PNP solve.
        @param variance       Mean positional variance of recent accepted poses (m²).
                              Pass 0.0 when BOT_ADAPTIVE_STDDEVS is off.
        @return [x, y, theta] standard deviations.
    */
    wpi::array<double, 3> computeStdDevs (double distanceMeters, int tagCount, double variance = 0.0) const;

    /** Retrieves all available vision measurements from the last update cycle.
     
        @warning Must be called **exactly once per periodic cycle**. Implementations
        clear and refill an internal buffer on each call — calling it more than once
        in the same cycle will return an empty vector on the second call, since the
        underlying camera results are consumed (unread) on the first.

        @param currentPose The drivetrain's current fused pose. Used for residual
               gating — any candidate more than `kMaxResidual` from this pose is
               rejected before it can reach `AddVisionMeasurement()`. Get this
               from `drivetrain.GetState().Pose` before calling.
     
        @return Const reference to the internal measurement buffer (0 or 1 entries);
                valid until the next call.
    */
    virtual void readMeasurements (const frc::Pose2d& currentPose) = 0;

    /** Processes a batch of pipeline results from one camera.
     
        Applies latency, tag-count, distance, field-bounds, and odometry-residual
        gating. Candidates that survive all checks are compared against each other
        and the single best one (most tags, then shortest distance) is appended to
        `_measurements`. Rejection counters are updated for every rejected result.

        Subclasses call this once per camera inside `getMeasurements()` after
        fetching raw results — that is the *only* thing subclasses need to do.

        @param cameraName  Display name of the source camera (e.g. "FL").
        @param estimator   The `PhotonPoseEstimator` associated with this camera.
        @param results     Raw pipeline results from `GetAllUnreadResults()`.
        @param currentPose The drivetrain's current fused pose for residual gating.
    */
    void processResults (const std::string& cameraName,
                         photon::PhotonPoseEstimator& estimator,
                         std::vector<photon::PhotonPipelineResult>& results,
                         const frc::Pose2d& currentPose);
};

/** Vision system configuration constants.
 
    Contains fixed configuration parameters for camera mounting positions,
    turret geometry, and AprilTag field information. These values are
    shared between real hardware and simulation.
*/
struct Camera {
    enum Index : int { Turret,
                       Left,
                       Right };
};

/** Camera names (must match PhotonVision configuration) */
constexpr const std::array<const char*, 3> kCameraNames = { "Turret", "Left", "Right" };

/** Camera mounting positions relative to robot center. */
constexpr const std::array<frc::Transform3d, 3> kRobotToCamera = {
    // Turret: rear-left corner, facing straight backward, pitched up 10°
    frc::Transform3d {
        frc::Translation3d { -13.74_in, 2.24_in, 11.35_in },
        frc::Rotation3d { 0_deg, -22_deg, 180_deg } },
    // Left: 3_in inset from back frame edge → X = -13.75_in + 3_in = -10.75_in.
    // 13.74_in from robot centerline (+Y, left side). 7.25_in above ground.
    // Facing exactly left (yaw +90°). Pitched 8° upward (lens toward ceiling).
    // 4° roll clockwise when viewed from the lens (right-hand: -4° around X).
    frc::Transform3d {
        frc::Translation3d { -10.75_in, 13.74_in, 7.25_in },
        frc::Rotation3d { -4_deg, 8_deg, 90_deg } },
    // Right: 1.5 inches from right edge of robot. 2.453 inches from back of robot. 7.625 inches from the ground.
    // Robot frame: +X = forward, +Y = left, origin = robot center. Frame is 27.5 x 27.5 in (half = 13.75_in).
    // X: back edge (-13.75_in) + 2.453_in inset = -11.297_in
    // Y: right edge (-13.75_in) + 1.5_in inset  = -12.25_in
    // Z: 7.625_in above ground
    // Yaw -90_deg = facing right (-Y). Pitch +20_deg = tilted upward.
    frc::Transform3d {
        frc::Translation3d { -11.297_in, -12.25_in, 7.625_in },
        frc::Rotation3d { 0_deg, 20_deg, -90_deg } }
};

/** Turret pivot point location in robot frame (meters from robot center). */
constexpr const frc::Translation2d kTurretPivotInRobot { -7.5_in, 0.75_in };
} // namespace indy::vision
