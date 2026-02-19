#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

namespace subsystems {

/** Turret shooter subsystem with auto-aim capability.
 
    Controls a turret rotation motor for aiming and shooter flywheel motor(s).
    Supports both auto-aim mode (using robot pose from vision) and manual control.
*/
class Turret : public frc2::SubsystemBase {
public:
    Turret();

    void Periodic() override;

    // Command factories
    frc2::CommandPtr aimAtTargetCommand(std::function<frc::Pose2d()> robotPoseSupplier,
                                         std::function<frc::Pose2d()> targetPoseSupplier);
    frc2::CommandPtr manualRotateCommand(std::function<double()> speedSupplier);
    frc2::CommandPtr spinUpCommand();
    frc2::CommandPtr stopCommand();

    frc2::CommandPtr shootCommand();
    frc2::CommandPtr manualShootCommand();

    // Manual control
    void setRotationVelocity(units::turns_per_second_t velocity);
    void setRotationDutyCycle(double dutyCycle);  // For testing: -0.1 to 0.1
    void setShooterVelocity(units::turns_per_second_t velocity);
    void stopRotation();
    void stopShooter();
    void stop();

    // Auto-aim functions
    void enableAutoAim();
    void disableAutoAim();
    bool isAutoAimEnabled() const { return _autoAimEnabled; }
    void setTargetAngle(units::degree_t angle);
    
    // Status
    units::degree_t getCurrentAngle() const;
    units::turns_per_second_t getShooterVelocity() const;
    bool isAtTarget() const;
    bool isShooterReady() const;

private:
    // Rotation motor (positions turret)
    ctre::phoenix6::hardware::TalonFX _rotationMotor{
        config::integer("turret_rotation_device_id"),
        config::str("turret_rotation_can_bus")};
    
    // Shooter flywheel motor(s)
    ctre::phoenix6::hardware::TalonFX _shooterMotor{
        config::integer("turret_shooter_device_id"),
        config::str("turret_shooter_can_bus")};
    
    // Uptake motor (feeds game pieces into shooter)
    ctre::phoenix6::hardware::TalonFX _uptakeMotor{
        config::integer("turret_uptake_device_id"),
        config::str("turret_uptake_can_bus")};

    // Control requests (reusable)
    ctre::phoenix6::controls::PositionVoltage _positionRequest{0_tr};
    ctre::phoenix6::controls::VelocityVoltage _rotationVelocityRequest{0_tps};
    ctre::phoenix6::controls::VelocityVoltage _shooterVelocityRequest{0_tps};
    ctre::phoenix6::controls::VelocityVoltage _uptakeVelocityRequest{0_tps};
    ctre::phoenix6::controls::VoltageOut _voltageRequest{0_V};
    ctre::phoenix6::controls::DutyCycleOut _dutyCycleRequest{0.0};

    // State
    bool _autoAimEnabled = false;
    units::degree_t _targetAngle = 0_deg;

    // Constants
    static constexpr units::turns_per_second_t kShooterVelocity = 50_tps;  // TODO: tune
    static constexpr units::turns_per_second_t kShooterTolerance = 2_tps;
    static constexpr units::turns_per_second_t kUptakeVelocity = 100_tps;   // TODO: tune uptake speed
    static constexpr units::degree_t kAngleTolerance = 2_deg;
    static constexpr units::turns_per_second_t kManualRotationSpeed = 0.5_tps;
    
    // Gear ratio from motor to turret (motor rotations per turret rotation)
    static constexpr double kRotationGearRatio = 100.0;  // TODO: measure actual ratio

    void configureMotors();
    units::degree_t computeAimAngle(const frc::Pose2d& robotPose, 
                                     const frc::Pose2d& targetPose) const;
};

} // namespace subsystems
