#include "shaker.hpp"
#include <frc2/command/Commands.h>

using namespace ctre::phoenix6;

namespace indy {

Shaker::Shaker() {
    SetName("Shaker");
    configureMotor();
}

void Shaker::configureMotor() {
    configs::TalonFXConfiguration cfg = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(30_A)
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(60_A)
                .WithStatorCurrentLimitEnable(true)
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );

    _motor.GetConfigurator().Apply(cfg);
}

void Shaker::spin(double dutyCycle) {
    dutyCycle = std::clamp(dutyCycle, -1.0, 1.0);
    _motor.SetControl(_dutyCycleRequest.WithOutput(dutyCycle));
}

void Shaker::stop() {
    _motor.SetControl(_dutyCycleRequest.WithOutput(0.0));
}

frc2::CommandPtr Shaker::spinCommand(double dutyCycle) {
    return StartEnd(
        [this, dutyCycle] { spin(dutyCycle); },
        [this]            { stop(); }
    ).WithName("Shaker");
}

frc2::CommandPtr Shaker::oscillateCommand(double dutyCycle, units::second_t period) {
    return frc2::cmd::Sequence(
        RunOnce([this, dutyCycle] { spin(dutyCycle);  }),
        frc2::cmd::Wait(period),
        RunOnce([this, dutyCycle] { spin(-dutyCycle); }),
        frc2::cmd::Wait(period / 2.0)
    )
    .Repeatedly()
    .FinallyDo([this] { stop(); })
    .WithName("ShakerOscillate");
}

} // namespace indy
