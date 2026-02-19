#include "visiontest.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>
#include <cmath>

namespace test {

frc2::CommandPtr createVisionTrackingTest(subsystems::Turret* turret, 
                                           VisionIOSingle* vision) {
    std::cout << "[bot] creating vision test command\n";
    return frc2::cmd::Run([turret, vision] {        
        // Get vision measurements (for validation and telemetry)
        // auto measurements = vision->getMeasurements();
        
        // Update telemetry
        frc::SmartDashboard::PutString("VisionTest/Status", vision->getStatus());
        frc::SmartDashboard::PutString("VisionTest/Targets", vision->getLastTargets());
        frc::SmartDashboard::PutString("VisionTest/Rejections", vision->getRejectedCounts());
        
        // Get target yaw angle for simple proportional control
        double targetYaw = vision->getBestTargetYaw();
        
        // Debug: always output the raw yaw
        frc::SmartDashboard::PutNumber("VisionTest/RawTargetYaw", targetYaw);
        
        if (std::abs(targetYaw) < 0.5) {  // Increased deadband to 0.5 degrees
            // No targets or already centered - stop turret
            turret->stopRotation();
            frc::SmartDashboard::PutBoolean("VisionTest/Tracking", false);
            frc::SmartDashboard::PutNumber("VisionTest/DutyCycle", 0.0);
            return;
        }
        
        // Simple proportional control with increased gain
        // Positive yaw = target is to the right → rotate right (positive duty cycle)
        // Negative yaw = target is to the left → rotate left (negative duty cycle)
        double kP = 0.01;  // Increased from 0.003 for more aggressive tracking
        double dutyCycle = targetYaw * kP;
        
        // Clamp duty cycle to ±0.1
        dutyCycle = std::clamp(dutyCycle, -0.1, 0.1);
        
        // Apply to turret
        turret->setRotationDutyCycle(dutyCycle);
        
        // Telemetry
        frc::SmartDashboard::PutBoolean("VisionTest/Tracking", true);
        frc::SmartDashboard::PutNumber("VisionTest/TargetYaw", targetYaw);
        frc::SmartDashboard::PutNumber("VisionTest/DutyCycle", dutyCycle);
        // frc::SmartDashboard::PutNumber("VisionTest/MeasurementCount", measurements.size());
    }, {turret})
    .WithName("VisionTrackingTest")
    .FinallyDo([turret] { turret->stopRotation(); });
}

} // namespace test
