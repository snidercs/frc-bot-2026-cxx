#pragma once

#include <string>
#include <vector>
#include <array>

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
#include <wpi/array.h>
#include <frc/DriverStation.h>

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
    virtual ~VisionIO() = default;

    /** Retrieves all available vision measurements from the last update cycle.
     
        @warning Must be called **exactly once per periodic cycle**. Implementations
        clear and refill an internal buffer on each call — calling it more than once
        in the same cycle will return an empty vector on the second call, since the
        underlying camera results are consumed (unread) on the first.
     
        @return Const reference to the internal measurement buffer; valid until the next call.
    */
    virtual const std::vector<VisionMeasurement>& getMeasurements() = 0;

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
    static constexpr double          kMaxAmbiguity = 0.3;
    static constexpr units::second_t kMaxLatency   = 0.5_s;

    // ── Shared measurement buffer (cleared + refilled each cycle) ───────────
    std::vector<VisionMeasurement> _measurements;

    // ── Rejection counters ──────────────────────────────────────────────────
    int _rejectedNoTargets   = 0;
    int _rejectedStale       = 0;
    int _rejectedAmbiguous   = 0;
    int _rejectedOutOfBounds = 0;
    int _acceptedCount       = 0;

    /** Scales pose std devs with distance: closer = more trusted.
     
        @param distanceMeters Distance from camera to best tag in metres.
        @return [x, y, theta] standard deviations.
    */
    wpi::array<double, 3> computeStdDevs(double distanceMeters) const {
        double xy    = 0.01 + (distanceMeters * 0.05);
        double theta = 0.01 + (distanceMeters * 0.02);
        return {xy, xy, theta};
    }

    /** Processes a batch of pipeline results from one camera.
     
        Applies latency, ambiguity, and field-bounds gating. Accepted poses are
        appended to `_measurements` and published to SmartDashboard. Rejection
        counters are updated for every rejected result.

        Subclasses call this once per camera inside `getMeasurements()` after
        fetching raw results — that is the *only* thing subclasses need to do.

        @param cameraName  Display name of the source camera (e.g. "FL").
        @param estimator   The `PhotonPoseEstimator` associated with this camera.
        @param results     Raw pipeline results from `GetAllUnreadResults()`.
    */
    void processResults(const std::string& cameraName,
                        photon::PhotonPoseEstimator& estimator,
                        std::vector<photon::PhotonPipelineResult>& results)
    {
        for (auto& result : results) {
            if (!result.HasTargets()) {
                _rejectedNoTargets++;
                continue;
            }

            if (result.GetLatency() > kMaxLatency) {
                _rejectedStale++;
                continue;
            }

            if (result.GetBestTarget().GetPoseAmbiguity() > kMaxAmbiguity) {
                _rejectedAmbiguous++;
                continue;
            }

            auto estimatedPose = estimator.Update(result);
            if (!estimatedPose.has_value()) {
                _rejectedNoTargets++;
                continue;
            }

            auto bestTransform = result.GetBestTarget().GetBestCameraToTarget();
            double distance = units::math::sqrt(
                units::math::pow<2>(bestTransform.X()) +
                units::math::pow<2>(bestTransform.Y()) +
                units::math::pow<2>(bestTransform.Z())
            ).value();

            auto pose2d = estimatedPose->estimatedPose.ToPose2d();
            if (pose2d.X() < 0_m || pose2d.X() > 16.46_m ||
                pose2d.Y() < 0_m || pose2d.Y() > 8.21_m) {
                _rejectedOutOfBounds++;
                continue;
            }

            _measurements.push_back(VisionMeasurement{
                pose2d,
                estimatedPose->timestamp,
                computeStdDevs(distance),
                cameraName
            });

            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/X (m)",      pose2d.X().value());
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Y (m)",      pose2d.Y().value());
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Rot (deg)",  pose2d.Rotation().Degrees().value());

            _acceptedCount++;
        }

        frc::SmartDashboard::PutNumber("Vision/Accepted",             _acceptedCount);
        frc::SmartDashboard::PutNumber("Vision/Rejected OutOfBounds", _rejectedOutOfBounds);
    }
};

/** Vision system configuration constants.
 
    Contains fixed configuration parameters for camera mounting positions,
    turret geometry, and AprilTag field information. These values are
    shared between real hardware and simulation.
*/
namespace vision {
    /** Camera names (must match PhotonVision configuration) */
    constexpr const std::array<const char*, 4> kCameraNames = {"FL", "FR", "BL", "BR"};
    
    /** Camera mounting positions relative to robot center (measured in meters/degrees).
     
        Index order: FL=0, FR=1, BL=2, BR=3
        Cameras are fixed to chassis (not turret), angled ~45° outward.
        
        TODO: FR and BR cameras are not yet mounted — transforms are placeholders.
    */
    constexpr const std::array<frc::Transform3d, 4> kRobotToCamera = {
        // Front-Left camera: on front edge, 6cm from left side, 51cm high
        frc::Transform3d{
            frc::Translation3d{13.74_in, 11.5_in, 15_in},
            frc::Rotation3d{0_deg, 0_deg, 0_deg}
        },
        // Front-Right camera (not yet mounted — placeholder)
        frc::Transform3d{
            frc::Translation3d{9.84_in, -9.84_in, 19.69_in},
            frc::Rotation3d{0_deg, 0_deg, -45_deg}
        },
        // Back-Left camera: 3.5cm past back edge, 37cm from right side, 41cm high
        frc::Transform3d{
            frc::Translation3d{-15.00_in, -12.25_in, 11.35_in},
            frc::Rotation3d{0_deg, 0_deg, 180_deg}
        },
        // Back-Right camera
        frc::Transform3d{
            frc::Translation3d{-9.84_in, -9.84_in, 19.69_in},
            frc::Rotation3d{0_deg, 0_deg, -135_deg}
        }
    };
    
    /** Turret pivot point location in robot frame (meters from robot center).
     
        TODO: Measure and update with actual turret mounting location
    */
    constexpr const frc::Translation2d kTurretPivotInRobot{0.0_m, 0.0_m};
    
    /** AprilTag field layout for the current season.
     
        Loads the standard field layout from WPILib resources.
        Returns empty layout if load fails (caller should handle).
    */
    inline frc::AprilTagFieldLayout getFieldLayout() {
        // Load 2026 field layout
        return frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltAndyMark);
    }
    
    /** Valid goal tag IDs that we're allowed to aim at.
     
        TODO: Update based on game rules and alliance strategy.
        Example: For 2025 Reefscape, blue alliance scoring tags might be {1, 2, 3}
    */
    inline const std::vector<int> kGoalTagIds = {1, 2, 3, 4, 5, 6, 7, 8};
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
        if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue) {
            return frc::Translation2d{182.105_in, 158.845_in};
        }
        return frc::Translation2d{469.115_in, 158.845_in};
    }

} // namespace landmarks
