#include "climber.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;
using namespace ctre::phoenix6;

Climber::Climber() {
    SetName("Climber");
    configureMotor();
}

void Climber::configureMotor() {
    // Basic TalonFX configuration for duty cycle control
    configs::TalonFXConfiguration config = configs::TalonFXConfiguration{}
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
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Brake)  // Use brake mode for climber safety
        )
        .WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}
                .WithSensorToMechanismRatio(180.0)
        )
        .WithSoftwareLimitSwitch(
            ctre::phoenix6::configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable(true)
                .WithForwardSoftLimitThreshold(kForwardSoftLimit)
                .WithReverseSoftLimitEnable(true)
                .WithReverseSoftLimitThreshold(kReverseSoftLimit)
        );

    // Apply config
    m_motor.GetConfigurator().Apply(config);
    m_motor.SetPosition(0.0_tr);
    // Configure status signal update frequencies to prevent CAN stale errors
    // Set to 50 Hz for telemetry signals
    BaseStatusSignal::SetUpdateFrequencyForAll(
        50_Hz,
        m_motor.GetVelocity(),
        m_motor.GetSupplyCurrent(),
        m_motor.GetMotorVoltage()
    );
}

void Climber::Periodic() {
#if BOT_TRACE_SUBSYSTEMS
    // Telemetry for debugging and monitoring
    frc::SmartDashboard::PutNumber("Climber/Velocity (rps)", m_motor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Climber/Current (A)", m_motor.GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Climber/Voltage (V)", m_motor.GetMotorVoltage().GetValue().value());
#endif
}

void Climber::setDutyCycle(double dutyCycle) {
    m_motor.SetControl(m_dutyCycleRequest.WithOutput(dutyCycle));
}

void Climber::stop() {
    m_motor.SetControl(m_dutyCycleRequest.WithOutput(0.0));
}

// Command factories
frc2::CommandPtr Climber::climbCommand() {
    return Run([this] { setDutyCycle(kClimbDutyCycle); })
        .WithName("Climb")
        .FinallyDo([this] { stop(); });
}

frc2::CommandPtr Climber::lowerCommand() {
    return Run([this] { setDutyCycle(kLowerDutyCycle); })
        .WithName("Lower")
        .FinallyDo([this] { stop(); });
}

frc2::CommandPtr Climber::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("StopClimber");
}

frc2::CommandPtr Climber::disableSoftLimitsCommand() {
    return RunOnce([this] {
        m_motor.GetConfigurator().Apply(
            configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable(false)
                .WithReverseSoftLimitEnable(false)
        );
    }).WithName("DisableSoftLimits");
}

frc2::CommandPtr Climber::enableSoftLimitsAndResetCommand() {
    return RunOnce([this] {
        m_motor.SetPosition(0.0_tr);
        m_motor.GetConfigurator().Apply(
            configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable(true)
                .WithForwardSoftLimitThreshold(kForwardSoftLimit)
                .WithReverseSoftLimitEnable(true)
                .WithReverseSoftLimitThreshold(kReverseSoftLimit)
        );
    }).WithName("EnableSoftLimitsAndReset");
}
