#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/time.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "config.hpp"

// 9ft front right corner to back right corner of recepticle.
// speed 55 tps
// 3ft 10 inches recepticle

// 52 tps at 6 ft 10 inches
// 50 tps works at 6 ft 10 inches too.
// 52 tps at 8ft dead on.

namespace indy {

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

    // --- Shoot-on-the-move (SOTM) command factories ---

    /** Auto-aim with shoot-on-the-move lead compensation.
     
        Predicts the robot's future pose based on current velocity and aims at
        where the turret needs to point by the time the ball arrives. Also feeds
        a velocity hint to the rotation motor for smoother tracking.

        @param robotPoseSupplier  Supplier returning the current fused robot pose.
        @param targetPoseSupplier Supplier returning the target (hub) pose.
        @param speedsSupplier     Supplier returning the current robot-centric ChassisSpeeds.
    */
    frc2::CommandPtr sotmAimCommand(std::function<frc::Pose2d()> robotPoseSupplier,
                                    std::function<frc::Pose2d()> targetPoseSupplier,
                                    std::function<frc::ChassisSpeeds()> speedsSupplier);

    /** Shoot-on-the-move shoot command — adjusts shooter speed for radial velocity.
     
        Like shootAtDistanceCommand, but compensates the distance lookup for the
        robot's radial velocity relative to the target so the ball arrives at the
        correct speed even while the robot is moving toward/away from the hub.

        @param distanceFn      Supplier returning current distance to target (metres).
        @param radialVelFn     Supplier returning radial velocity toward target (m/s, positive = closing).
    */
    frc2::CommandPtr sotmShootCommand(std::function<units::meter_t()> distanceFn,
                                      std::function<units::meters_per_second_t()> radialVelFn);

    /** Spin up the shooter and run the uptake.
     
        @param distanceFn Optional supplier returning current distance to target (metres).
                          When provided, shooter velocity tracks distance continuously.
                          When null, falls back to `kShooterVelocity`.
    */
    frc2::CommandPtr shooterOnCommand(std::function<units::meter_t()> distanceFn = nullptr);
    frc2::CommandPtr shooterOffCommand();

    /** Teleop shoot command — continuously adjusts shooter speed as distance changes.
     
        @param distanceFn Supplier returning current distance to target (metres).
    */
    frc2::CommandPtr shootAtDistanceCommand(std::function<units::meter_t()> distanceFn);

    frc2::CommandPtr spinUpCommand();
    frc2::CommandPtr stopCommand();
    frc2::CommandPtr calibrateRotationZero();

    // Manual control
    void setRotationVelocity(units::turns_per_second_t velocity);
    void setRotationDutyCycle(double dutyCycle);  // For testing: -0.1 to 0.1
    void updateRotationControl(double operatorCommand);  // NEW: Handles hold/manual control
    void setShooterVelocity(units::turns_per_second_t velocity);
    void stopRotation();
    void stopShooter();
    void stopUptake();
    void stop();

    // Auto-aim functions
    void enableAutoAim();
    void disableAutoAim();
    bool isAutoAimEnabled() const { return _autoAimEnabled; }
    void setTargetAngle(units::degree_t angle);
    
    // Status
    /** Returns the current turret rotation angle from the cached sensor value. */
    units::degree_t getCurrentAngle() const;

    /** Returns the current shooter flywheel velocity from the cached sensor value. */
    units::turns_per_second_t getShooterVelocity() const;

    /** Returns the last commanded shooter target velocity (for sim state feedback). */
    units::turns_per_second_t cachedShooterTarget() const { return _cachedShooterTarget; }

    /** Returns the shooter motor (for sim state access in SimulationPeriodic). */
    ctre::phoenix6::hardware::TalonFX& shooterMotor() { return _shooterMotor; }

    /** Returns true if the turret rotation is within @c kAngleTolerance of the target angle. */
    bool isAtTarget() const;

    /** Returns true if the shooter flywheel is within @c kShooterTolerance of @c kShooterVelocity. */
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
    
    // Cached sensor values (updated in Periodic to avoid redundant CAN reads)
    units::degree_t _cachedAngle = 0_deg;
    units::turns_per_second_t _cachedShooterVelocity = 0_tps;
    units::turns_per_second_t _cachedShooterTarget = 0_tps;
    units::turns_per_second_t _cachedRotationVelocity = 0_tps;
    units::volt_t _cachedMotorVoltage = 0_V;
    units::ampere_t _cachedMotorCurrent = 0_A;

    // Position hold state (for locking when no operator input)
    bool _isHoldingPosition = false;
    units::turn_t _holdPosition = 0_tr;

    // Live-tunable scale factor applied to distance before shooter speed lookup.
    // Adjust via Elastic slider - we'll use this to figure out what scale we need for the competition balls vs. our balls
    double _distanceScale = 1.0;

    // Constants
    static constexpr units::turns_per_second_t kShooterVelocity = 54_tps;
    static constexpr units::turns_per_second_t kFallbackShooterVelocity = 52_tps;
    static constexpr units::turns_per_second_t kShooterTolerance = 5_tps;
    static constexpr units::turns_per_second_t kUptakeVelocity = 100_tps;
    static constexpr units::degree_t kAngleTolerance = 2_deg;
    static constexpr units::turns_per_second_t kManualRotationSpeed = 0.3_tps;
    
    // --- SOTM (shoot-on-the-move) constants ---
    // Tuning-phase values: adjust via SmartDashboard, then bake into constexpr.
    static constexpr units::second_t kDefaultLookaheadTime = 0.25_s;
    static constexpr double kDefaultTurretFFGain = 1.0;
    static constexpr units::meters_per_second_t kMaxSOTMSpeed = 3.5_mps;
    static constexpr units::turns_per_second_t kMaxTurretFF = 0.5_tps;

    // Live-tunable SOTM gains (SmartDashboard sliders during tuning phase)
    double _sotmLookahead = kDefaultLookaheadTime.value();
    double _sotmFFGain = kDefaultTurretFFGain;

    void configureMotors();
    void setTargetPosition(units::turn_t position);
    void setTargetPositionWithFF(units::turn_t position, units::turns_per_second_t velocityFF);
    units::turn_t computeAimPosition(const frc::Pose2d& robotPose, 
                                     const frc::Pose2d& targetPose) const;

    /** Compute SOTM-compensated aim position + turret velocity feedforward.
     
        Predicts the robot pose forward by the lookahead time, computes the aim
        angle from that predicted pose, and derives the turret angular velocity
        needed to track the moving aim point.

        @param robotPose   Current fused robot pose (field-relative).
        @param targetPose  Target (hub) pose (field-relative).
        @param fieldSpeeds Field-relative chassis speeds.
        @return Pair of {aim position in turret turns, velocity feedforward in tps}.
    */
    std::pair<units::turn_t, units::turns_per_second_t>
    computeAimWithSOTM(const frc::Pose2d& robotPose,
                       const frc::Pose2d& targetPose,
                       const frc::ChassisSpeeds& fieldSpeeds) const;

    /** Returns shooter flywheel velocity for a given distance to the hub.
        When auto aiming is off, this will return a fallback, fixed, velocity.

        Linear interpolation between two measured calibration points:
        - 1.75 m (~5 ft 9 in): 46 tps
        - 5.5 m  (~18 ft):     65.2 tps
        Clamps to [46, 65.2] tps outside the calibrated range.

        @param distance Distance from robot to hub in metres.
        @return Target flywheel velocity in turns per second.
    */
    units::turns_per_second_t velocityFromDistance(units::meter_t distance) const;
};

} // namespace subsystems
