#include "turret.hpp"
#include "vision.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>

using namespace subsystems;
using namespace ctre::phoenix6;

Turret::Turret() {
    SetName("Turret");
    configureMotors();
}

void Turret::configureMotors() {
    // Rotation motor configuration (position control for aiming)
    configs::TalonFXConfiguration rotationConfig = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(30_A)
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(60_A)
                .WithStatorCurrentLimitEnable(true)
        )
        .WithVoltage(
            configs::VoltageConfigs{}
                .WithPeakForwardVoltage(12_V)
                .WithPeakReverseVoltage(-12_V)
        )
        .WithSlot0(
            configs::Slot0Configs{}
                .WithKP(24.0)     // TODO: tune PID gains
                .WithKI(0.0)
                .WithKD(0.2)
                .WithKV(0.12)
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Brake)
        )
        .WithFeedback(
            configs::FeedbackConfigs{}
                .WithSensorToMechanismRatio(kRotationGearRatio)
        );
    
    // Shooter motor configuration (velocity control for flywheel)
    configs::TalonFXConfiguration shooterConfig = configs::TalonFXConfiguration{}
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
                .WithKP(0.2)      // TODO: tune flywheel PID
                .WithKI(0.0)
                .WithKD(0.0)
                .WithKV(0.12)
                .WithKS(0.25)     // Static friction feedforward
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
    
    // Apply configs
    _rotationMotor.GetConfigurator().Apply(rotationConfig);
    // _shooterMotor.GetConfigurator().Apply(shooterConfig);
    
    // // Configure status signal update frequencies
    // BaseStatusSignal::SetUpdateFrequencyForAll(
    //     50_Hz,
    //     _rotationMotor.GetPosition(),
    //     _rotationMotor.GetVelocity(),
    //     _rotationMotor.GetSupplyCurrent(),
    //     _shooterMotor.GetVelocity(),
    //     _shooterMotor.GetSupplyCurrent()
    // );
    
    // Optimize CAN bus utilization
    _rotationMotor.OptimizeBusUtilization();
    // _shooterMotor.OptimizeBusUtilization();
}

void Turret::Periodic() {
    // Telemetry
    frc::SmartDashboard::PutBoolean("Turret/Auto Aim Enabled", _autoAimEnabled);
    frc::SmartDashboard::PutNumber("Turret/Current Angle (deg)", getCurrentAngle().value());
    frc::SmartDashboard::PutNumber("Turret/Target Angle (deg)", _targetAngle.value());
    frc::SmartDashboard::PutNumber("Turret/Rotation Velocity (rps)", _rotationMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("Turret/Shooter Velocity (rps)", getShooterVelocity().value());
    frc::SmartDashboard::PutBoolean("Turret/At Target", isAtTarget());
    frc::SmartDashboard::PutBoolean("Turret/Shooter Ready", isShooterReady());
    
    // Debug: Motor status
    frc::SmartDashboard::PutNumber("Turret/Motor Voltage", _rotationMotor.GetMotorVoltage().GetValue().value());
    frc::SmartDashboard::PutNumber("Turret/Motor Current", _rotationMotor.GetSupplyCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Turret/Motor Position", _rotationMotor.GetPosition().GetValue().value());
}

void Turret::setRotationVelocity(units::turns_per_second_t velocity) {
    _rotationMotor.SetControl(_rotationVelocityRequest.WithVelocity(velocity));
}

void Turret::setRotationDutyCycle(double dutyCycle) {
    // Clamp to safe range for testing
    dutyCycle = std::clamp(dutyCycle, -0.1, 0.1);
    
    // Debug output
    frc::SmartDashboard::PutNumber("Turret/Commanded Duty Cycle", dutyCycle);
    
    _rotationMotor.SetControl(_dutyCycleRequest.WithOutput(dutyCycle));
}

void Turret::setShooterVelocity(units::turns_per_second_t velocity) {
    // _shooterMotor.SetControl(_shooterVelocityRequest.WithVelocity(velocity));
}

void Turret::stopRotation() {
    _rotationMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stopShooter() {
    // _shooterMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stop() {
    stopRotation();
    stopShooter();
}

void Turret::enableAutoAim() {
    _autoAimEnabled = true;
}

void Turret::disableAutoAim() {
    _autoAimEnabled = false;
}

void Turret::setTargetAngle(units::degree_t angle) {
    _targetAngle = angle;
    // Convert degrees to rotations for position control
    units::turn_t targetRotations = angle / 360.0;
    _rotationMotor.SetControl(_positionRequest.WithPosition(targetRotations));
}

units::degree_t Turret::getCurrentAngle() const {
    // Convert motor rotations back to degrees
    // Need to cast away const to call GetPosition() - Phoenix 6 API limitation
    auto rotations = const_cast<ctre::phoenix6::hardware::TalonFX&>(_rotationMotor).GetPosition().GetValue();
    return rotations * 360.0;
}

units::turns_per_second_t Turret::getShooterVelocity() const {
    // Need to cast away const to call GetVelocity() - Phoenix 6 API limitation
    return 0_rad_per_s; //const_cast<ctre::phoenix6::hardware::TalonFX&>(_shooterMotor).GetVelocity().GetValue();
}

bool Turret::isAtTarget() const {
    auto error = units::math::abs(getCurrentAngle() - _targetAngle);
    return error < kAngleTolerance;
}

bool Turret::isShooterReady() const {
    auto velocityError = units::math::abs(getShooterVelocity() - kShooterVelocity);
    return velocityError < kShooterTolerance;
}

units::degree_t Turret::computeAimAngle(const frc::Pose2d& robotPose, 
                                         const frc::Pose2d& targetPose) const {
    // Get target position in field coordinates
    auto targetTranslation = targetPose.Translation();
    
    // Transform target to robot frame
    auto robotToTarget = targetTranslation - robotPose.Translation();
    auto robotToTargetInRobotFrame = robotToTarget.RotateBy(-robotPose.Rotation());
    
    // Subtract turret pivot offset (from vision constants)
    auto turretPivot = vision::kTurretPivotInRobot;
    auto pivotToTarget = robotToTargetInRobotFrame - turretPivot;
    
    // Calculate angle using atan2
    auto angleRad = units::radian_t{std::atan2(pivotToTarget.Y().value(), pivotToTarget.X().value())};
    return units::degree_t{angleRad};
}

// Command factories
frc2::CommandPtr Turret::aimAtTargetCommand(std::function<frc::Pose2d()> robotPoseSupplier,
                                              std::function<frc::Pose2d()> targetPoseSupplier) {
    return Run([this, robotPoseSupplier, targetPoseSupplier] {
        if (_autoAimEnabled) {
            auto robotPose = robotPoseSupplier();
            auto targetPose = targetPoseSupplier();
            auto aimAngle = computeAimAngle(robotPose, targetPose);
            setTargetAngle(aimAngle);
        }
    })
    .WithName("AimAtTarget");
}

frc2::CommandPtr Turret::manualRotateCommand(std::function<double()> speedSupplier) {
    return Run([this, speedSupplier] {
        if (!_autoAimEnabled) {
            double speed = speedSupplier();
            setRotationVelocity(speed * kManualRotationSpeed);
        }
    })
    .WithName("ManualRotate")
    .FinallyDo([this] { stopRotation(); });
}

frc2::CommandPtr Turret::spinUpCommand() {
    return Run([this] { setShooterVelocity(kShooterVelocity); })
        .WithName("SpinUp");
}

frc2::CommandPtr Turret::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("StopTurret");
}

frc2::CommandPtr Turret::shootCommand() {
    return frc2::cmd::Sequence(
        // Spin up flywheel
        Run([this] { setShooterVelocity(kShooterVelocity); })
            .Until([this] { return isShooterReady(); })
            .WithTimeout(2.0_s),
        // Wait for aim to be on target
        frc2::cmd::WaitUntil([this] { return isAtTarget() && isShooterReady(); })
            .WithTimeout(1.0_s),
        // TODO: Add command to feed note into shooter
        frc2::cmd::Wait(0.5_s),  // Placeholder for feeding time
        // Stop shooter
        RunOnce([this] { stopShooter(); })
    )
    .WithName("Shoot");
}
