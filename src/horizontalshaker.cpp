#include "horizontalshaker.hpp"
#include <frc2/command/Commands.h>

using namespace ctre::phoenix6;

namespace indy {

HorizontalShaker::HorizontalShaker() {
    SetName("HorizontalShaker");
    configureMotor();
}

void HorizontalShaker::configureMotor() {
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
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );

    _motor.GetConfigurator().Apply(cfg);
}

void HorizontalShaker::spin(double dutyCycle) {
    dutyCycle = std::clamp(dutyCycle, -1.0, 1.0);
    _motor.SetControl(_dutyCycleRequest.WithOutput(dutyCycle));
}

void HorizontalShaker::stop() {
    _motor.SetControl(_dutyCycleRequest.WithOutput(0.0));
}

frc2::CommandPtr HorizontalShaker::spinCommand(double dutyCycle) {
    return StartEnd(
        [this, dutyCycle] { spin(dutyCycle); },
        [this]            { stop(); }
    ).WithName("HorizontalShaker");
}

frc2::CommandPtr HorizontalShaker::oscillateCommand(double dutyCycle, units::second_t period) {
    return frc2::cmd::Sequence(
        RunOnce([this, dutyCycle] { spin(dutyCycle);  }),
        frc2::cmd::Wait(period),
        RunOnce([this] { stop(); }),
        frc2::cmd::Wait(period),
        RunOnce([this, dutyCycle] { spin(-dutyCycle); }),
        frc2::cmd::Wait(period),
        RunOnce([this] { stop(); })
    )
    .Repeatedly()
    .FinallyDo([this] { stop(); })
    .WithName("HorizontalShakerOscillate");
}

} // namespace indy
