#include "turret.hpp"
#include "vision.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <numbers>

using namespace ctre::phoenix6;

namespace indy {

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
    
        ).WithFeedback(
            ctre::phoenix6::configs::FeedbackConfigs{}
                .WithSensorToMechanismRatio(10.0)
        )
        .WithSoftwareLimitSwitch(
            ctre::phoenix6::configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable(true)
                .WithForwardSoftLimitThreshold(0.25_tr)
                .WithReverseSoftLimitEnable(true)
                .WithReverseSoftLimitThreshold(-0.05542_tr)
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
    _rotationMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    _rotationMotor.SetPosition(0.25_tr);

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

    // Always-on: key shooter values for tuning
    frc::SmartDashboard::PutNumber("Turret/Shooter Velocity (tps)", _cachedShooterVelocity.value());
    frc::SmartDashboard::PutBoolean("Turret/Shooter Ready", isShooterReady());
    frc::SmartDashboard::PutNumber("Turret/Shooter Target (tps)", _cachedShooterTarget.value());

#if BOT_TRACE_SUBSYSTEMS
    // Verbose telemetry (all values from cache — no additional CAN reads)
    frc::SmartDashboard::PutBoolean("Turret/Auto Aim Enabled", _autoAimEnabled);
    frc::SmartDashboard::PutNumber("Turret/Current Angle (deg)", _cachedAngle.value());
    frc::SmartDashboard::PutNumber("Turret/Target Angle (deg)", _targetAngle.value());
    frc::SmartDashboard::PutNumber("Turret/Rotation Velocity (rps)", _cachedRotationVelocity.value());
    frc::SmartDashboard::PutBoolean("Turret/At Target", isAtTarget());
    frc::SmartDashboard::PutNumber("Turret/Motor Voltage", _cachedMotorVoltage.value());
    frc::SmartDashboard::PutNumber("Turret/Motor Current", _cachedMotorCurrent.value());
    frc::SmartDashboard::PutNumber("Turret/Motor Position", _rotationMotor.GetPosition().GetValue().value());
#endif
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

void Turret::updateRotationControl(double operatorCommand) {
    // Deadband check for "zero" input
    constexpr double kDeadband = 0.05;
    bool hasInput = std::abs(operatorCommand) > kDeadband;
    
    if (hasInput) {
        // Operator is actively controlling - use percent output
        _isHoldingPosition = false;
        
        // Scale command to a reasonable output range (e.g., -0.3 to 0.3 for safe manual control)
        constexpr double kMaxOutput = 0.3;
        double scaledOutput = std::clamp(operatorCommand, -1.0, 1.0) * kMaxOutput;
        
        _rotationMotor.SetControl(_dutyCycleRequest.WithOutput(scaledOutput));
        
    } else {
        // No operator input - hold current position
        if (!_isHoldingPosition) {
            // LATCH the current position (only once when entering hold mode)
            // This prevents continuously resetting the setpoint which would fight
            // any small corrections the controller is making
            _holdPosition = _rotationMotor.GetPosition().GetValue();
            _isHoldingPosition = true;
        }
        
        // Use position control to actively hold the latched position
        // This resists drift from gravity/momentum
        _rotationMotor.SetControl(_positionRequest.WithPosition(_holdPosition));
    }
    
    // Telemetry for debugging
    frc::SmartDashboard::PutBoolean("Turret/Holding Position", _isHoldingPosition);
    frc::SmartDashboard::PutNumber("Turret/Hold Position (turns)", _holdPosition.value());
}

void Turret::setShooterVelocity(units::turns_per_second_t velocity) {
    _cachedShooterTarget = velocity;
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
    auto velocityError = units::math::abs(getShooterVelocity() - _cachedShooterTarget);
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
            // Use the new position-hold control logic
            updateRotationControl(speed);
        }
    })
    .WithName("ManualRotate");
    // NOTE: No FinallyDo needed - updateRotationControl handles hold automatically
}

frc2::CommandPtr Turret::spinUpCommand() {
    return Run([this] { setShooterVelocity(kShooterVelocity); })
        .WithName("SpinUp");
}

frc2::CommandPtr Turret::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("StopTurret");
}

units::turns_per_second_t Turret::velocityFromDistance(units::meter_t distance) const {
    // Two measured calibration points (from comments at top of file):
    //   2.08 m (6 ft 10 in) → 50 tps
    //   2.74 m (9 ft)       → 55 tps
    constexpr units::meter_t kNearDist  = 2.378_m;
    constexpr units::meter_t kFarDist   = 5_m;
    constexpr double         kNearSpeed = 50.0;
    constexpr double         kFarSpeed  = 65.0;

    double t = (distance - kNearDist) / (kFarDist - kNearDist);
    t = std::clamp(t, 0.0, 1.0);
    return units::turns_per_second_t{kNearSpeed + t * (kFarSpeed - kNearSpeed)};
}

frc2::CommandPtr Turret::shooterOnCommand(std::function<units::meter_t()> distanceFn) {
    // Resolve velocity: use distance supplier when available, else fixed constant.
    auto targetVelocity = [this, distanceFn]() -> units::turns_per_second_t {
        return distanceFn ? velocityFromDistance(distanceFn()) : kShooterVelocity;
    };

    // Warm path: already at speed → start uptake immediately.
    // Cold path: spin up, wait until ready, then start uptake.
    // Either way, the flywheel keeps tracking distance while the command runs.
    auto warm = frc2::cmd::Sequence(
        RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        RunOnce([this] {
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        })
    );

    auto cold = frc2::cmd::Sequence(
        RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        RunOnce([this] {
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        })
    );

    return frc2::cmd::Either(
        std::move(warm),
        std::move(cold),
        [this] { return isShooterReady(); }
    ).WithName("ShooterOn");
}

frc2::CommandPtr Turret::shootAtDistanceCommand(std::function<units::meter_t()> distanceFn) {
    return frc2::cmd::Sequence(
        // Spin up with initial distance-based speed, wait until ready
        RunOnce([this, distanceFn] { setShooterVelocity(velocityFromDistance(distanceFn())); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        // Once ready: continuously track distance and run uptake
        Run([this, distanceFn] {
            setShooterVelocity(velocityFromDistance(distanceFn()));
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        })
    )
    .FinallyDo([this] {
        stopUptake();
        stopShooter();
    })
    .WithName("ShootAtDistance");
}

frc2::CommandPtr Turret::shooterOffCommand() {
    return RunOnce([this] {
        stopUptake();
        stopShooter();
    }).WithName("ShooterOff");
}

frc2::CommandPtr Turret::shootCommand() {
    return frc2::cmd::Sequence(
        shooterOnCommand(),
        frc2::cmd::Idle()
    )
    .FinallyDo([this] { 
        stopUptake();
        stopShooter(); 
    })
    .WithName("ManualShoot");
}

frc2::CommandPtr Turret::calibrateRotationZero() {
    return RunOnce([this] {
        // Calibrate the zero point
        stopRotation();
        _holdPosition = 0_tr;
        _rotationMotor.SetPosition(_holdPosition);
        _isHoldingPosition = true;
    }).WithName("ZeroRotation");
}

}
