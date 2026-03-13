#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <units/voltage.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace indy {

/** Shaker subsystem — spins a single TalonFX motor at a fixed duty cycle.
 
    Duty cycle control (DutyCycleOut) maps -1.0 to full reverse and +1.0
    to full forward, relative to the 12V bus. No closed-loop involved.
*/
class Shaker : public frc2::SubsystemBase {
public:
    Shaker();

    /** Spin at the given duty cycle [-1, 1]. */
    void spin(double dutyCycle);

    /** Stop the motor (coast). */
    void stop();

    /** Command: hold the shaker at @p dutyCycle while the command runs.
        Stops on end.
     
        @param dutyCycle Duty cycle in [-1, 1]. Positive = forward.
    */
    frc2::CommandPtr spinCommand(double dutyCycle);

    /** Command: oscillate forward/reverse at @p dutyCycle, switching every @p period.
        Stops on end.

        @param dutyCycle Duty cycle magnitude [0, 1].
        @param period    Time spent in each direction before switching.
    */
    frc2::CommandPtr oscillateCommand(double dutyCycle, units::second_t period = 0.12_s);

private:
    ctre::phoenix6::hardware::TalonFX _motor{
        config::integer("shaker_device_id"),
        config::str("shaker_can_bus")};

    ctre::phoenix6::controls::DutyCycleOut _dutyCycleRequest{0.0};

    void configureMotor();
};

} // namespace indy
