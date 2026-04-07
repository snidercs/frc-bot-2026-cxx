#include "intake.hpp"
#include "units/time.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

using namespace ctre::phoenix6;

namespace indy {

Intake::Intake()
    : kFeedVoltage{config::number("intake_voltage") * 1.0_V}
{
    SetName("Intake");
    configureMotors();

    // Default command: keep the intake extended (down) at all times.
    SetDefaultCommand(Run([this] { extend(); }).WithName("IntakeDefault"));
}

void Intake::configureMotors() {
    // ── Pitch motors (OTB left / right) ────────────────────────────────
    // Duty-cycle controlled for simple extend/retract intake movement.
    configs::TalonFXConfiguration pitchConfig = configs::TalonFXConfiguration{}
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
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Brake)
        );

    // Right pitch motor mirrors left but inverted
    configs::TalonFXConfiguration pitchRightConfig = configs::TalonFXConfiguration{pitchConfig}
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Brake)
        );

    _otbLeft.GetConfigurator().Apply(pitchConfig);
    _otbRight.GetConfigurator().Apply(pitchRightConfig);

    // ── Feed motor ─────────────────────────────────────────────────────
    configs::TalonFXConfiguration feedConfig = configs::TalonFXConfiguration{}
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
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );

    _feedMotor.GetConfigurator().Apply(feedConfig);

    // ── Status signal update frequencies ───────────────────────────────
    BaseStatusSignal::SetUpdateFrequencyForAll(
        50_Hz,
        _otbLeft.GetPosition(),
        _otbLeft.GetVelocity(),
        _otbLeft.GetSupplyCurrent(),
        _otbRight.GetPosition(),
        _otbRight.GetVelocity(),
        _otbRight.GetSupplyCurrent(),
        _feedMotor.GetVelocity(),
        _feedMotor.GetSupplyCurrent()
    );

    _otbLeft.OptimizeBusUtilization();
    _otbRight.OptimizeBusUtilization();
    _feedMotor.OptimizeBusUtilization();
}

void Intake::Periodic() {
#if BOT_TRACE_SUBSYSTEMS
    frc::SmartDashboard::PutNumber("Intake/PitchL Pos (tr)",
        _otbLeft.GetPosition().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/PitchR Pos (tr)",
        _otbRight.GetPosition().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/PitchL Current (A)",
        _otbLeft.GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/PitchR Current (A)",
        _otbRight.GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Feed Velocity (rps)",
        _feedMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Feed Current (A)",
        _feedMotor.GetSupplyCurrent().GetValue().value());
#endif
}

// ── Pitch control ──────────────────────────────────────────────────────

void Intake::extend() {
    _otbLeft.SetControl(_dutyCycleRequest.WithOutput(-kExtendDutyCycle));
    _otbRight.SetControl(_dutyCycleRequest.WithOutput(kExtendDutyCycle));
}

void Intake::retract() {
    _otbLeft.SetControl(_dutyCycleRequest.WithOutput(kRetractDutyCycle));
    _otbRight.SetControl(_dutyCycleRequest.WithOutput(-kRetractDutyCycle));
}

void Intake::stopPitch() {
    _otbLeft.SetControl(_dutyCycleRequest.WithOutput(0.0));
    _otbRight.SetControl(_dutyCycleRequest.WithOutput(0.0));
}

// ── Feed (roller) control ──────────────────────────────────────────────

void Intake::feed() {
    _feedMotor.SetControl(_voltageRequest.WithOutput(kFeedVoltage));
}

void Intake::eject() {
    _feedMotor.SetControl(_voltageRequest.WithOutput(kEjectVoltage));
}

void Intake::stopFeed() {
    _feedMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

// ── Convenience ────────────────────────────────────────────────────────

void Intake::stop() {
    stopPitch();
    stopFeed();
}

// ── Command factories ──────────────────────────────────────────────────

frc2::CommandPtr Intake::intakeCommand() {
    return Run([this] { feed(); })
        .WithName("Intake")
        .FinallyDo([this] { stopFeed(); });
}

frc2::CommandPtr Intake::ejectCommand() {
    return Run([this] { eject(); })
        .WithName("Eject")
        .FinallyDo([this] { stopFeed(); });
}

frc2::CommandPtr Intake::startCommand() {
    return RunOnce([this] { feed(); })
        .WithName("IntakeStart");
}

frc2::CommandPtr Intake::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("IntakeStop");
}

frc2::CommandPtr Intake::retractCommand() {
    return Run([this] { retract(); })
        .WithName("IntakeRetract")
        .FinallyDo([this] { extend(); });
}

frc2::CommandPtr Intake::stutterCommand(units::time::second_t duration) {
    auto startTime = std::make_shared<units::second_t>(0_s);
    if (duration <= 0_s)
        duration = units::time::second_t(config::number("intake_stutter_length"));

    return RunOnce([startTime] {
        *startTime = frc::Timer::GetFPGATimestamp();
    })
    .AndThen(Run([this, startTime] {
        double t = frc::Timer::GetFPGATimestamp().value();
        if (std::fmod(t, 0.5) < 0.25)
            feed();
        else
            stopFeed();
    })
    .Until([startTime, duration] {
        return (frc::Timer::GetFPGATimestamp() - *startTime) >= duration;
    }))
    .FinallyDo([this] { stop(); })
    .WithName("IntakeStutter");
}

} // namespace indy
