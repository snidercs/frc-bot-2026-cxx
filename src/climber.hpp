#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace indy {

class Climber : public frc2::SubsystemBase {
public:
    Climber();

    void Periodic() override;

    // Command factories
    frc2::CommandPtr retractCommand();
    frc2::CommandPtr extendCommand();
    frc2::CommandPtr stopCommand();
    frc2::CommandPtr disableSoftLimitsCommand();
    frc2::CommandPtr enableSoftLimitsAndResetCommand();

    // Manual control
    void setDutyCycle(double dutyCycle);
    void stop();

private:
    // Single Kraken x60 motor for climber
    ctre::phoenix6::hardware::TalonFX m_motor{
        config::integer("climber_device_id"),
        config::str("climber_can_bus")};

    // Control request (reusable)
    ctre::phoenix6::controls::DutyCycleOut m_dutyCycleRequest{0.0};

    // Constants
    static constexpr double kRetractDutyCycle = 0.99;   // Positive = retracting
    static constexpr double kExtendDutyCycle = -0.99;   // Negative = extending
    static constexpr units::turn_t kForwardSoftLimit = 0.0_tr;
    static constexpr units::turn_t kReverseSoftLimit = -3.089355_tr;

    void configureMotor();
};

} // namespace indy
