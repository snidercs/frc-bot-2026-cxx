#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/array.h>
#include <frc/DriverStation.h>

namespace indy {

/** A single vision pose measurement from a camera.
 
    Contains the estimated robot pose, timestamp, standard deviations for
    pose uncertainty, and the source camera that produced the measurement.
*/
struct VisionMeasurement {
    /** The estimated robot pose on the field */
    frc::Pose2d pose;
    
    /** The timestamp when this measurement was captured */
    units::second_t timestamp;
    
    /** Standard deviations for pose uncertainty [x, y, theta] */
    wpi::array<double, 3> stdDevs;
    
    /** Source camera name (FL, FR, BL, BR) */
    std::string source;
};

/** Interface for vision measurement sources (real hardware or simulation).
 
    Provides a hardware abstraction layer for vision systems, allowing the
    same code to work with both PhotonVision cameras and simulation.

    ## Extending VisionIO
    Common pipeline logic lives here so subclasses stay thin. The only thing
    a subclass must supply is *how to get raw results* for each camera — call
    `processResults()` with those results and everything else (gating, std-dev
    scaling, SmartDashboard publishing, rejection counting) is handled here.

    To add new shared behaviour, add it here rather than duplicating it in
    both `VisionMulti` and `VisionSim`.
*/
class VisionIO {
public:
    VisionIO();
    virtual ~VisionIO() = default;

    struct Candidate {
        VisionMeasurement measurement;
        int    tagCount;
        double distance;
    };
    
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
    virtual const std::vector<VisionMeasurement>& getMeasurements(
        const frc::Pose2d& currentPose) = 0;

    /** Gets a human-readable status string for debugging.
     
        @return Status information about the vision system
    */
    virtual std::string getStatus() { return "VisionIO base class"; }

    /** Gets the most recent target information from all cameras.
     
        @return String describing detected targets (tag IDs, distances, etc.)
    */
    virtual std::string getLastTargets() { return "No targets"; }

    /** Gets counts of rejected measurements for debugging.
     
        @return String with rejection statistics (stale, ambiguous, etc.)
    */
    virtual std::string getRejectedCounts();

protected:
    // ── Gating constants ────────────────────────────────────────────────────
    static constexpr double          kMaxAmbiguity          = 0.2;
    static constexpr units::second_t kMaxLatency            = 0.25_s;
    static constexpr int             kMinTagsForSingleSolve = 2;
    static constexpr double          kMaxTagDistance        = 4.0;  // metres
    static constexpr units::meter_t  kMaxResidual           = 0.6_m;
    static constexpr units::meter_t  kLooseResidual         = 1.5_m;
    // Maximum physically plausible robot speed between two accepted vision poses.
    // Any implied velocity above this is treated as a bad solve and rejected.
    static constexpr units::meters_per_second_t kMaxImpliedVelocity = 5.0_mps;

    // ── Shared measurement buffer (cleared + refilled each cycle) ───────────
    std::vector<VisionMeasurement> _measurements;
    std::vector<Candidate> _candidates;

    // ── Rejection counters ──────────────────────────────────────────────────
    int _rejectedNoTargets   = 0;
    int _rejectedStale       = 0;
    int _rejectedAmbiguous   = 0;
    int _rejectedOutOfBounds = 0;
    int _rejectedResidual    = 0;
    int _rejectedVelocity    = 0;
    int _acceptedCount       = 0;

    uint32_t _dropouts = 0;

    // ── Per-camera velocity gate state ───────────────────────────────────────
    // Tracks the last committed accepted pose and its timestamp for each camera
    // so the velocity gate can reject solves that imply physically impossible motion.
    //
    // TODO(next-year): All gate state, constants, and processResults() logic are
    // good candidates to factor out into a shared TelemetryKit / VisionUtils library
    // so they can be reused across seasons without copying this file. The natural
    // boundary is a composable VisionGate interface with per-gate state encapsulation.
    struct CameraGateState {
        frc::Pose2d     lastPose{};
        units::second_t lastTime{ 0_s };

#if BOT_ADAPTIVE_STDDEVS
        // Rolling window for Option F adaptive stdDev scaling.
        static constexpr int kWindowSize = 6;
        std::array<frc::Translation2d, kWindowSize> recentPositions{};
        int windowCount{0};
        int windowHead{0};
#endif
    };
    std::unordered_map<std::string, CameraGateState> _cameraState;

    /** Scales pose std devs by distance and tag count: closer + more tags = more trusted.
     
        @param distanceMeters Distance from camera to nearest tag in metres.
        @param tagCount       Number of tags used in the PNP solve.
        @param variance       Mean positional variance of recent accepted poses (m²).
                              Pass 0.0 when BOT_ADAPTIVE_STDDEVS is off.
        @return [x, y, theta] standard deviations.
    */
    wpi::array<double, 3> computeStdDevs(double distanceMeters, int tagCount,
                                         double variance = 0.0) const {
        // Conservative base: further away = less trust
        double xy = 0.2 + (distanceMeters * 0.07);
        // Reward good multi-tag geometry
        if (tagCount > 2) xy *= 0.75;
#if BOT_ADAPTIVE_STDDEVS
        // Inflate stdDevs when recent measurements have been inconsistent.
        // kVarianceThreshold is the m² spread that begins scaling. kMaxScale caps it.
        static constexpr double kVarianceThreshold = 0.05;
        static constexpr double kMaxScale          = 3.0;
        const double scale = std::clamp(variance / kVarianceThreshold, 1.0, kMaxScale);
        xy *= scale;
#endif
        // theta: very high so vision never overrides the gyro heading
        double theta = 9999.0;
        return {xy, xy, theta};
    }

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
    void processResults(const std::string& cameraName,
                        photon::PhotonPoseEstimator& estimator,
                        std::vector<photon::PhotonPipelineResult>& results,
                        const frc::Pose2d& currentPose);
};

/** Vision system configuration constants.
 
    Contains fixed configuration parameters for camera mounting positions,
    turret geometry, and AprilTag field information. These values are
    shared between real hardware and simulation.
*/
namespace vision {
    /** Camera names (must match PhotonVision configuration) */
    constexpr const std::array<const char*, 2> kCameraNames = {"FL", "BL"};

    /** Camera mounting positions relative to robot center.
     
        Index order: FL=0, BL=1
        TODO: Add FR and BR once physically mounted and measured.
    */
    constexpr const std::array<frc::Transform3d, 2> kRobotToCamera = {
        // Front-Left: forward-left corner, facing straight forward
        frc::Transform3d{
            frc::Translation3d{13.74_in, 11.5_in, 15_in},
            frc::Rotation3d{0_deg, 0_deg, 0_deg}
        },
        // Back-Left: rear-left corner, facing straight backward, pitched up 10°
        frc::Transform3d{
            frc::Translation3d{-13.74_in, 2.24_in, 11.35_in},
            frc::Rotation3d{0_deg, -10_deg, 180_deg}
        }
    };
    
    /** Turret pivot point location in robot frame (meters from robot center).
     
        TODO: Measure and update with actual turret mounting location
    */
    constexpr const frc::Translation2d kTurretPivotInRobot{-7.5_in, 0.75_in};
    
    /** AprilTag field layout for the current season.
     
        Loads the standard field layout from WPILib resources.
        Returns empty layout if load fails (caller should handle).
    */
    inline frc::AprilTagFieldLayout fieldLayout() {
        // Load 2026 field layout
        return frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);
    }
}

/** Fixed field landmark positions for the 2026 game.
 
    Coordinates are in the WPILib field coordinate system (origin = blue
    alliance wall, x+ toward red alliance, y+ toward the left side when
    viewed from blue alliance).
    
    All positions sourced from the official 2026 field drawings.
*/
namespace landmarks {

    /** Returns the hub (goal) position for the current alliance.
     
        The hub x-coordinate mirrors across the field centre depending on
        alliance colour; the y-coordinate is always the same.
        Defaults to the red alliance position when alliance is unknown.
        
        @return The hub centre as a 2D field translation
    */
    inline frc::Translation2d hubPosition() {
        const auto alliance = frc::DriverStation::GetAlliance();
        return (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue)
            ? frc::Translation2d{182.11_in, 158.84_in}
            : frc::Translation2d{469.11_in, 158.84_in};
    }

} // namespace landmarks
}
