#pragma once

#include <frc2/command/CommandPtr.h>
#include "turret.hpp"
#include "visionsingle.hpp"
#include <memory>

namespace test {

/** Creates a simple vision-based turret tracking test command.
 
    Uses VisionIOSingle to detect AprilTags and commands the turret
    to point in the direction of detected tags. Useful for hardware
    validation before full system integration.
    
    @param turret The turret subsystem to control
    @param vision The single-camera vision system
    @return Command that tracks visible AprilTags
*/
frc2::CommandPtr createVisionTrackingTest(subsystems::Turret* turret, 
                                           VisionIOSingle* vision);

} // namespace test
