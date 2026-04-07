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
 *      between stowed (up) and deployed (down) positions.
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
    /** Spin feed while held; stop feed on release. Intake stays extended. */
    frc2::CommandPtr intakeCommand();
    /** Spin feed in reverse while held; stop on release. */
    frc2::CommandPtr ejectCommand();
    /** One-shot: start the feed rollers (no pitch movement). */
    frc2::CommandPtr startCommand();
    /** One-shot: stop all motors. */
    frc2::CommandPtr stopCommand();
    /** Retract intake while held; re-extends on release. */
    frc2::CommandPtr retractCommand();
    /** Stutter the feed on/off for a duration, then stop. */
    frc2::CommandPtr stutterCommand(units::time::second_t duration = 0_s);

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
    static constexpr units::volt_t kEjectVoltage = 2_V;

    /** Duty cycle to extend/lower the intake (OTBLeft gets negative, OTBRight gets positive). */
    static constexpr double kExtendDutyCycle = 0.05;
    /** Duty cycle to retract/raise the intake (OTBLeft gets positive, OTBRight gets negative). */
    static constexpr double kRetractDutyCycle = 0.15;
};

} // namespace indy
