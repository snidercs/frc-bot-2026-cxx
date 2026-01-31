#include "generated/TunerConstants.h"
#include "subsystems/drivetrain.hpp"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
