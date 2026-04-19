#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace indy {

/** Over-the-bumper (OTB) intake subsystem.
 *
 *  Three motors:
 *    - OTBLeft / OTBRight — synchronised pitch pair that rotates the intake
 *      between up and down positions.
 *    - Intake (feed) — spins the rollers to ingest game pieces.
 *
 *  The intake is held extended (down) by default and only retracts when
 *  explicitly commanded to do so.
 */
class Intake : public frc2::SubsystemBase {
public:
    Intake();

    void Periodic() override;

    // ── Pitch control ────────────────────────────────────────────────────
    /** Extend the intake to the down (intake) position. */
    void extend();
    /** Retract the intake to the up (stored) position. */
    void retract();
    /** Stop the pitch motors. */
    void stopPitch();

    // ── Feed (roller) control ──────────────────────────────────────────
    /** Run the feed rollers at the configured intake voltage. */
    void feed();
    /** Run the feed rollers in reverse to eject. */
    void eject();
    /** Stop the feed rollers. */
    void stopFeed();

    // ── Convenience ────────────────────────────────────────────────────
    /** Stop all motors (pitch + feed). */
    void stop();

    // ── Command factories ──────────────────────────────────────────────
    /** Spin feed while held. */
    frc2::CommandPtr intakeCommand();
    /** Spin feed in reverse while held. */
    frc2::CommandPtr ejectCommand();
    /** Start the feed rollers, continues to run until stopped.*/
    frc2::CommandPtr startCommand();
    /** Stop all motors. */
    frc2::CommandPtr stopCommand();
    /** Retract intake while held; re-extends on release. */
    frc2::CommandPtr retractCommand();
    /** Stutter the feed on/off for a duration, then stop. */
    frc2::CommandPtr stutterCommand(units::time::second_t duration = 0_s);
    /** Extend the intake until fully down, then finish. */
    frc2::CommandPtr extendCommand();
    /** Continuously oscillate the intake up/down to agitate game pieces.
     *
     *  Alternates between extending and retracting the pitch motors on a
     *  configurable cycle.  Intended to run alongside the shooter so that
     *  game pieces are jiggled into the uptake while scoring.
     */
    frc2::CommandPtr agitateCommand();

    /** Disable soft limits on both pitch motors. */
    frc2::CommandPtr disableSoftLimitsCommand();
    /** Re-zero encoders and re-enable soft limits on both pitch motors. */
    frc2::CommandPtr enableSoftLimitsAndResetCommand();

private:
    void configureMotors();

    // ── Motors ──────────────────────────────────────────────────────────
    ctre::phoenix6::hardware::TalonFX _otbLeft{
        config::integer("otb_left_device_id"),
        config::str("otb_left_can_bus")};
    ctre::phoenix6::hardware::TalonFX _otbRight{
        config::integer("otb_right_device_id"),
        config::str("otb_right_can_bus")};
    ctre::phoenix6::hardware::TalonFX _feedMotor{
        config::integer("intake_device_id"),
        config::str("intake_can_bus")};

    // ── Control requests (reusable, zero-alloc in periodic) ────────────
    ctre::phoenix6::controls::VoltageOut _voltageRequest{0_V};
    ctre::phoenix6::controls::DutyCycleOut _dutyCycleRequest{0.0};

    // ── Constants ──────────────────────────────────────────────────────
    const units::volt_t kFeedVoltage;
    static constexpr units::volt_t kEjectVoltage = -4_V;

    /** Duty cycle to extend/lower the intake (OTBLeft gets negative, OTBRight gets positive). */
    static constexpr double kExtendDutyCycle = 0.15;
    /** Duty cycle to retract/raise the intake (OTBLeft gets positive, OTBRight gets negative). */
    static constexpr double kRetractDutyCycle = 0.30;

    /** Duration of each extend phase during agitation (seconds). */
    static constexpr units::second_t kAgitateExtendTime = 0.20_s;
    /** Duration of each retract phase during agitation (seconds). */
    static constexpr units::second_t kAgitateRetractTime = 0.15_s;

    // ── Soft limits (motor rotations, starting from 0 = up/stowed) ─────
    static constexpr units::turn_t kLeftForwardLimit  =  0.0_tr;        // home (up)
    static constexpr units::turn_t kLeftReverseLimit  = -2.305078_tr;   // fully extended (down)
    static constexpr units::turn_t kRightForwardLimit =  2.170801_tr;   // fully extended (down)
    static constexpr units::turn_t kRightReverseLimit =  0.0_tr;        // home (up)
};

} // namespace indy
