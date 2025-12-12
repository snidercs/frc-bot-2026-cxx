#include "generated/TunerConstants.h"
#include "CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
