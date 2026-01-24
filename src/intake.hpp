#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace subsystems {

class Intake : public frc2::SubsystemBase {
public:
    Intake();

    void Periodic() override;

    // Command factories
    frc2::CommandPtr intakeCommand();
    frc2::CommandPtr ejectCommand();
    frc2::CommandPtr stopCommand();

    // Manual control
    void setVoltage(units::volt_t voltage);
    void setVelocity(units::turns_per_second_t velocity);
    void stop();

private:
    // Two Kraken x44 motors
    ctre::phoenix6::hardware::TalonFX m_topMotor{config::INTAKE_TOP_MOTOR_ID, "rio"};
    ctre::phoenix6::hardware::TalonFX m_bottomMotor{config::INTAKE_BOTTOM_MOTOR_ID, "rio"};

    // Control requests (reusable)
    ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
    ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};

    // Constants
    static constexpr units::volt_t kIntakeVoltage = 8_V;
    static constexpr units::volt_t kEjectVoltage = -6_V;
    static constexpr units::turns_per_second_t kIntakeVelocity = 50_tps;

    void configureMotors();
};

} // namespace subsystems
