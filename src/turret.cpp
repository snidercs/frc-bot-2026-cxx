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
        )
        .WithSoftwareLimitSwitch(
            configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitThreshold(0.5_tr)   // +180 degrees
                .WithForwardSoftLimitEnable(true)
                .WithReverseSoftLimitThreshold(-0.5_tr)  // -180 degrees
                .WithReverseSoftLimitEnable(true)
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
    
    // Uptake motor configuration (velocity control for feeding)
    configs::TalonFXConfiguration uptakeConfig = configs::TalonFXConfiguration{}
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
                .WithKP(0.2)      // TODO: tune uptake PID
                .WithKI(0.0)
                .WithKD(0.0)
                .WithKV(0.12)
                .WithKS(0.2)      // Static friction feedforward
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
    
    // Apply configs
    _rotationMotor.GetConfigurator().Apply(rotationConfig);
    _shooterMotor.GetConfigurator().Apply(shooterConfig);
    _uptakeMotor.GetConfigurator().Apply(uptakeConfig);
    
    // // Configure status signal update frequencies
    BaseStatusSignal::SetUpdateFrequencyForAll(
        50_Hz,
        _rotationMotor.GetPosition(),
        _rotationMotor.GetVelocity(),
        _rotationMotor.GetSupplyCurrent(),
        _shooterMotor.GetVelocity(),
        _shooterMotor.GetSupplyCurrent(),
        _uptakeMotor.GetVelocity(),
        _uptakeMotor.GetSupplyCurrent()
    );
    
    // Optimize CAN bus utilization
    _rotationMotor.OptimizeBusUtilization();
    _shooterMotor.OptimizeBusUtilization();
    _uptakeMotor.OptimizeBusUtilization();
}

void Turret::Periodic() {
    // Cache all sensor values once per cycle to minimize CAN bus reads
    _cachedAngle = _rotationMotor.GetPosition().GetValue() * 360.0;
    _cachedShooterVelocity = _shooterMotor.GetVelocity().GetValue();
    _cachedRotationVelocity = _rotationMotor.GetVelocity().GetValue();
    _cachedMotorVoltage = _rotationMotor.GetMotorVoltage().GetValue();
    _cachedMotorCurrent = _rotationMotor.GetSupplyCurrent().GetValue();
    
    // Telemetry (all values from cache — no additional CAN reads)
    frc::SmartDashboard::PutBoolean("Turret/Auto Aim Enabled", _autoAimEnabled);
    frc::SmartDashboard::PutNumber("Turret/Current Angle (deg)", _cachedAngle.value());
    frc::SmartDashboard::PutNumber("Turret/Target Angle (deg)", _targetAngle.value());
    frc::SmartDashboard::PutNumber("Turret/Rotation Velocity (rps)", _cachedRotationVelocity.value());
    frc::SmartDashboard::PutNumber("Turret/Shooter Velocity (rps)", _cachedShooterVelocity.value());
    frc::SmartDashboard::PutBoolean("Turret/At Target", isAtTarget());
    frc::SmartDashboard::PutBoolean("Turret/Shooter Ready", isShooterReady());
    
    // Debug: Motor status
    frc::SmartDashboard::PutNumber("Turret/Motor Voltage", _cachedMotorVoltage.value());
    frc::SmartDashboard::PutNumber("Turret/Motor Current", _cachedMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Turret/Motor Position", _cachedAngle.value() / 360.0);
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
    _shooterMotor.SetControl(_shooterVelocityRequest.WithVelocity(velocity));
}

void Turret::stopRotation() {
    _rotationMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stopShooter() {
    _shooterMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stopUptake() {
    _uptakeMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stop() {
    stopRotation();
    stopShooter();
    stopUptake();
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
    // units::turn_t{degree_t} performs automatic unit conversion (90_deg → 0.25_tr)
    units::turn_t targetRotations{angle};
    _rotationMotor.SetControl(_positionRequest.WithPosition(targetRotations));
}

units::degree_t Turret::getCurrentAngle() const {
    return _cachedAngle;
}

units::turns_per_second_t Turret::getShooterVelocity() const {
    return _cachedShooterVelocity;
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

frc2::CommandPtr Turret::manualShootCommand() {
    return frc2::cmd::Sequence(
        // Step 1: Spin up shooter flywheels
        RunOnce([this] {
            setShooterVelocity(kShooterVelocity);
        }),
        
        // Step 2: Wait 1 second for shooter to spin up
        frc2::cmd::Wait(1.0_s),
        
        // Step 3: Run uptake motor to feed balls into shooter
        Run([this] {
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        })
    )
    .FinallyDo([this] {
        // On button release: stop uptake immediately, let shooter coast down
        stopUptake();
        stopShooter();
    })
    .WithName("ManualShoot");
}
