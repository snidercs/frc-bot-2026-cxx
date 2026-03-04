#pragma once

#include <string>
#include <vector>
#include <array>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>
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
    virtual std::string getRejectedCounts() { return "No rejection data"; }
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
            frc::Translation3d{13.74_in, 11.38_in, 20.08_in},
            frc::Rotation3d{0_deg, 0_deg, 0_deg}
        },
        // Front-Right camera (not yet mounted — placeholder)
        frc::Transform3d{
            frc::Translation3d{9.84_in, -9.84_in, 19.69_in},
            frc::Rotation3d{0_deg, 0_deg, -45_deg}
        },
        // Back-Left camera: 3.5cm past back edge, 37cm from right side, 41cm high
        frc::Transform3d{
            frc::Translation3d{-15.12_in, 0.83_in, 16.14_in},
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
