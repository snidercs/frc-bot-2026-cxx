#include "intake.hpp"
#include "units/time.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

using namespace ctre::phoenix6;

namespace indy {

Intake::Intake()
    : kIntakeVoltage {config::number("intake_voltage") * 1.0_V}
{
    SetName("Intake");
    configureMotors();
}

void Intake::configureMotors() {
    // Top motor configuration
    configs::TalonFXConfiguration topConfig = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(40_A)
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(80_A)
                .WithStatorCurrentLimitEnable(true)
        )
        .WithVoltage(
            configs::VoltageConfigs{}
                .WithPeakForwardVoltage(12_V)
                .WithPeakReverseVoltage(-12_V)
        )
        .WithSlot0(
            configs::Slot0Configs{}
                .WithKP(0.1)
                .WithKI(0.0)
                .WithKD(0.0)
                .WithKV(0.12)
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
    
    // Bottom motor configuration (inverted)
    configs::TalonFXConfiguration bottomConfig = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(40_A)
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(80_A)
                .WithStatorCurrentLimitEnable(true)
        )
        .WithVoltage(
            configs::VoltageConfigs{}
                .WithPeakForwardVoltage(12_V)
                .WithPeakReverseVoltage(-12_V)
        )
        .WithSlot0(
            configs::Slot0Configs{}
                .WithKP(0.1)
                .WithKI(0.0)
                .WithKD(0.0)
                .WithKV(0.12)
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
    
    // Apply configs
    m_topMotor.GetConfigurator().Apply(topConfig);
    m_bottomMotor.GetConfigurator().Apply(bottomConfig);
    
    // Configure status signal update frequencies to prevent CAN stale errors
    // Set to 50 Hz for telemetry signals (velocity, current)
    BaseStatusSignal::SetUpdateFrequencyForAll(
        50_Hz,
        m_topMotor.GetVelocity(),
        m_topMotor.GetSupplyCurrent(),
        m_bottomMotor.GetVelocity(),
        m_bottomMotor.GetSupplyCurrent()
    );
    
    // Optimize CAN bus utilization after setting update frequencies
    // This reduces the default update rates for unused signals
    m_topMotor.OptimizeBusUtilization();
    m_bottomMotor.OptimizeBusUtilization();
    
    // Optional: Make bottom motor follow top motor
    // m_bottomMotor.SetControl(controls::Follower{14, true}); // Follow ID 14, opposite direction
}

void Intake::Periodic() {
#if BOT_TRACE_SUBSYSTEMS
    // Telemetry for debugging and monitoring
    frc::SmartDashboard::PutNumber("Intake/Top Velocity (rps)", m_topMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Bottom Velocity (rps)", m_bottomMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Top Current (A)", m_topMotor.GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Bottom Current (A)", m_bottomMotor.GetSupplyCurrent().GetValue().value());
#endif
}

void Intake::setVoltage(units::volt_t voltage) {
    m_topMotor.SetControl(m_voltageRequest.WithOutput(voltage));
    m_bottomMotor.SetControl(m_voltageRequest.WithOutput(voltage));
}

void Intake::setVelocity(units::turns_per_second_t velocity) {
    m_topMotor.SetControl(m_velocityRequest.WithVelocity(velocity));
    m_bottomMotor.SetControl(m_velocityRequest.WithVelocity(velocity));
}

void Intake::stop() {
    m_topMotor.SetControl(m_voltageRequest.WithOutput(0_V));
    m_bottomMotor.SetControl(m_voltageRequest.WithOutput(0_V));
}

// Command factories
frc2::CommandPtr Intake::startCommand() {
    return RunOnce([this] { setVoltage(kIntakeVoltage); })
        .WithName("IntakeStart");
}

frc2::CommandPtr Intake::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("IntakeStop");
}

frc2::CommandPtr Intake::intakeCommand() {
    return Run([this] { setVoltage(kIntakeVoltage); })
        .WithName("Intake")
        .FinallyDo([this] { stop(); });
}

frc2::CommandPtr Intake::ejectCommand() {
    return Run([this] { setVoltage(kEjectVoltage); })
        .WithName("Eject")
        .FinallyDo([this] { stop(); });
}

frc2::CommandPtr Intake::stutterCommand (units::time::second_t duration) {
    // Run the intake in 0.25s on / 0.25s off cycles for 6 seconds then stop.
    // Uses a captured start time to determine elapsed duration and self-terminate.
    auto startTime = std::make_shared<units::second_t>(0_s);
    if (duration <= 0_s)
        duration = units::time::second_t (config::number ("intake_stutter_length"));

    return RunOnce([startTime] {
        *startTime = frc::Timer::GetFPGATimestamp();
    })
    .AndThen(Run([this, startTime] {
        double t = frc::Timer::GetFPGATimestamp().value();
        if (std::fmod(t, 0.5) < 0.25)
            setVoltage(kIntakeVoltage);
        else
            stop();
    })
    .Until([startTime, duration] {
        return (frc::Timer::GetFPGATimestamp() - *startTime) >= duration;
    }))
    .FinallyDo([this] { stop(); })
    .WithName("IntakeStutter");
}

}
