#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace indy {

class Intake : public frc2::SubsystemBase {
public:
    Intake();

    void Periodic() override;

    // Command factories
    frc2::CommandPtr startCommand();
    frc2::CommandPtr stopCommand();
    frc2::CommandPtr intakeCommand();
    frc2::CommandPtr ejectCommand();


    /** Runs the intake in 0.25s on / 0.25s off cycles, then stops.
     
        @param duration How long to run the stutter cycle before stopping.
                        If <= 0, the duration is read from `intake_stutter_length` in config.lua.
    */
    frc2::CommandPtr stutterCommand(units::time::second_t duration = 0_s);

    // Manual control
    void setVoltage(units::volt_t voltage);
    void setVelocity(units::turns_per_second_t velocity);
    void stop();

private:
    // Two Kraken x44 motors
    ctre::phoenix6::hardware::TalonFX m_topMotor{
        config::integer("intake_top_device_id"),
        config::str("intake_top_can_bus")};
    ctre::phoenix6::hardware::TalonFX m_bottomMotor{
        config::integer("intake_bottom_device_id"),
        config::str("intake_bottom_can_bus")};

    // Control requests (reusable)
    ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};
    ctre::phoenix6::controls::VelocityVoltage m_velocityRequest{0_tps};

    // Constants
    const units::volt_t kIntakeVoltage;
    static constexpr units::volt_t kEjectVoltage = 2_V;
    static constexpr units::turns_per_second_t kIntakeVelocity = 50_tps;

    void configureMotors();
};

} // namespace subsystems
