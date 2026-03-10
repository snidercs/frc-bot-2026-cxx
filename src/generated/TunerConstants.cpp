#include "generated/TunerConstants.h"
#include "drivetrain.hpp"

indy::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
