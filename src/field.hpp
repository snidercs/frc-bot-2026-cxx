#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Translation2d.h>
#include <frc/DriverStation.h>

/** Fixed field landmark positions for the 2026 game.
 
    Coordinates are in the WPILib field coordinate system (origin = blue
    alliance wall, x+ toward red alliance, y+ toward the left side when
    viewed from blue alliance).
    
    All positions sourced from the official 2026 field drawings.
*/
namespace indy::field {
/** Field length 2026 (left to right) */
inline constexpr units::meter_t length() { return 16.535_m; }
/** Field width (bottom to top) */
inline constexpr units::meter_t width() { return 8.069_m; }
/** Out of bounds margin */
inline constexpr units::meter_t margin() { return 0.75_m; }

/** AprilTag field layout for the current season.
     
        Loads the standard field layout from WPILib resources.
        Returns empty layout if load fails (caller should handle).
    */
inline frc::AprilTagFieldLayout layout()
{
    // Load 2026 field layout
    return frc::AprilTagFieldLayout::LoadField (frc::AprilTagField::k2026RebuiltAndyMark);
}

/** Returns the hub (goal) position for the current alliance.
     
        The hub x-coordinate mirrors across the field centre depending on
        alliance colour; the y-coordinate is always the same.
        Defaults to the red alliance position when alliance is unknown.
        
        @return The hub centre as a 2D field translation
    */
inline frc::Translation2d hubPosition()
{
    const auto alliance = frc::DriverStation::GetAlliance();
    return (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue)
               ? frc::Translation2d { 182.11_in, 158.84_in }
               : frc::Translation2d { 469.11_in, 158.84_in };
}

} // namespace indy::field
