#pragma once

#include "frc/geometry/Pose2d.h"
#include "units/length.h"
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
/** Field length along the alliance-to-alliance axis (X). */
inline constexpr units::meter_t length() { return 16.535_m; }
/** Field width perpendicular to the alliance walls (Y). */
inline constexpr units::meter_t width() { return 8.069_m; }
/** Out of bounds safety margin. */
inline constexpr units::meter_t margin() { return 0.75_m; }

/** X distance from the alliance back wall to the lob-zone aim point.
 
    When the robot is in the lob zone (past the hub on the opponent's side),
    the turret aims at a point this far from the robot's own alliance wall
    so that the ball arcs and lands near the hub for pickup.
*/
inline constexpr units::inch_t kLobAimWallOffset = 181.56_in + 25_in;
/** Y coordinate of the field centre line (parallel to the alliance walls). */
inline constexpr units::inch_t halfWidth() { return 158.32_in; }
// Tunable: additional X offset from kLobAimWallOffset toward the hub.
static constexpr units::meter_t kLobXOffset = 2.0_m;
// Tunable: lateral distance from the field centre to the side landing target.
static constexpr units::meter_t kLobYOffset = 3.0_m;

/** Loads the AprilTag field layout for the current season.
 
    Returns the standard WPILib 2026 layout.
    Caller should handle the case where the load fails (empty layout).
*/
inline frc::AprilTagFieldLayout layout()
{
    return frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);
}

/** Returns the hub (goal) centre position for the current alliance.
 
    The hub X mirrors across the field centre depending on alliance colour.
    Defaults to the red alliance position when alliance is unknown.
        
    @return Hub centre as a 2D field translation.
*/
inline frc::Translation2d hubPosition()
{
    const auto alliance = frc::DriverStation::GetAlliance();
    return (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue)
               ? frc::Translation2d { 182.11_in, 158.32_in }
               : frc::Translation2d { 469.11_in, 158.32_in };
}

/** Returns the aim target for the turret given the current robot pose.
 
    ## Normal zone
    When the robot is on its own side of the hub (has not crossed the hub's
    X coordinate), the turret aims directly at the hub for a standard shot.

    ## Lob zone
    When the robot has crossed past the hub into opponent territory, a direct
    shot is not viable. Instead the turret aims at a side work area next to
    the hub so the ball arcs and lands on the ground for pickup. The robot
    then drives back, picks the ball up, and shoots from a good position.

    Left or right side is chosen by comparing the robot's Y to the field
    centre line — robot above centre aims left, below centre aims right.

    ### Tunable offsets (inside `aimPosition`)
    - `kLobXOffset` — additional X distance beyond `kLobAimWallOffset`,
      adjusts how far from the own alliance wall the landing target is.
    - `kLobYOffset` — lateral distance from the field centre line to the
      left/right landing target.

    @param robotPose Current robot pose in field coordinates.
    @return The 2D field translation the turret should aim at.
*/
inline frc::Translation2d aimPosition(const frc::Pose2d& robotPose) {
    

    const auto alliance = frc::DriverStation::GetAlliance();
    const bool isBlue = alliance.has_value() &&
                        alliance.value() == frc::DriverStation::Alliance::kBlue;

    const auto hub        = hubPosition();
    const auto robotX     = robotPose.X();
    const auto robotY     = robotPose.Y();
    const auto fieldCentreY = units::meter_t { halfWidth() };

    // "Beyond the hub" — robot has crossed the hub's X toward the opponent's
    // side. Blue robots cross in the +X direction, red in the -X direction.
    const bool beyondHub = isBlue ? (robotX > hub.X()) : (robotX < hub.X());

    if (!beyondHub) {
        // Normal scoring zone — aim straight at the hub.
        return hub;
    }

    // Lob zone — aim beside the hub so the ball lands near it for pickup.

    // X: kLobAimWallOffset is measured from the own alliance back wall.
    //    Blue wall is at X=0, red wall is at X=length().
    units::meter_t targetX = isBlue
        ? units::meter_t { kLobAimWallOffset } + kLobXOffset
        : length() - units::meter_t { kLobAimWallOffset } - kLobXOffset;

    // Y: aim left (+Y) when the robot is in the upper half of the field,
    //    right (-Y) when in the lower half.
    const bool isLeftSide = robotY > fieldCentreY;
    units::meter_t targetY = isLeftSide
        ? fieldCentreY + kLobYOffset
        : fieldCentreY - kLobYOffset;

    return { targetX, targetY };
}

} // namespace indy::field
