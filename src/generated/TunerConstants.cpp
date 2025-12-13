#include "generated/TunerConstants.h"
#include "drivetrain.hpp"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return { DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight };
}
