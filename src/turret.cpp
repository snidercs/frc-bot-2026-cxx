#include "turret.hpp"
#include "vision.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <numbers>

using namespace ctre::phoenix6;

namespace indy {

constexpr units::turn_t kMinPos = -0.73_tr; // Dead zone / cable wrap point; ~310° travel to kMaxPos
constexpr units::turn_t kMaxPos =  0.25_tr;

// Motion Magic cruise velocity for rotation.
// Halved automatically when a large wrap-around move is detected.
constexpr units::turns_per_second_t kRotationCruiseVelocity     = 1.5_tps;
constexpr units::turns_per_second_squared_t kRotationAcceleration = 3.0_tr_per_s_sq;

// Moves larger than this threshold (in turns) are considered wrap-arounds
// and will use half the normal cruise velocity.
constexpr units::turn_t kLargeMoveThreshold = 0.2_tr;

Turret::Turret()
    : kUptakeStallAmps{config::number("turret_uptake_stall_amps") * 1.0_A},
      kUptakeReverseTime{config::number("turret_uptake_reverse_time") * 1.0_s},
      kUptakeReverseVoltage{config::number("turret_uptake_reverse_voltage") * 1.0_V}
{
    SetName("Turret");
    configureMotors();
    frc::SmartDashboard::SetDefaultNumber("Turret/Distance Scale", _distanceScale);
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
                .WithKA(0.01)     // Required for Motion Magic torque/acceleration feedforward
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Brake)
    
        ).WithFeedback(
            ::configs::FeedbackConfigs{}
                .WithSensorToMechanismRatio(10.0)
        )
        .WithMotionMagic(
            configs::MotionMagicConfigs{}
                .WithMotionMagicCruiseVelocity(kRotationCruiseVelocity)
                .WithMotionMagicAcceleration(kRotationAcceleration)
        )
        .WithSoftwareLimitSwitch(
            ::configs::SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable(true)
                .WithForwardSoftLimitThreshold(kMaxPos)
                .WithReverseSoftLimitEnable(true)
                .WithReverseSoftLimitThreshold(kMinPos)
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
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
    
    // Uptake motor configuration (velocity control for feeding)
    configs::TalonFXConfiguration uptakeConfig = configs::TalonFXConfiguration{}
        .WithCurrentLimits(
            configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(30_A)
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
                .WithKP(0.2)      // TODO: tune uptake PID
                .WithKI(0.0)
                .WithKD(0.0)
                .WithKV(0.12)
                .WithKS(0.2)      // Static friction feedforward
        )
        .WithMotorOutput(
            configs::MotorOutputConfigs{}
                .WithInverted(signals::InvertedValue::Clockwise_Positive)
                .WithNeutralMode(signals::NeutralModeValue::Coast)
        );
        
    
    // Apply configs
    _rotationMotor.GetConfigurator().Apply(rotationConfig);
    _rotationMotor.SetNeutralMode(::signals::NeutralModeValue::Brake);
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
        _uptakeMotor.GetSupplyCurrent(),
        _uptakeMotor.GetStatorCurrent()
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
    _cachedUptakeStatorCurrent = _uptakeMotor.GetStatorCurrent().GetValue();

    // Always-on: key shooter values for tuning
    frc::SmartDashboard::PutNumber("Turret/Shooter Velocity (tps)", _cachedShooterVelocity.value());
    frc::SmartDashboard::PutBoolean("Turret/Shooter Ready", isShooterReady());
    frc::SmartDashboard::PutNumber("Turret/Shooter Target (tps)", _cachedShooterTarget.value());

    // Read distance scale from Elastic slider (clamped for safety)
    _distanceScale = std::clamp(
        frc::SmartDashboard::GetNumber("Turret/Distance Scale", 1.0), 0.5, 1.5);

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
#if BOT_TRACE_SUBSYSTEMS
    // Debug output
    frc::SmartDashboard::PutNumber("Turret/Commanded Duty Cycle", dutyCycle);
#endif    
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
#if BOT_TRACE_SUBSYSTEMS
    // Telemetry for debugging
    frc::SmartDashboard::PutBoolean("Turret/Holding Position", _isHoldingPosition);
    frc::SmartDashboard::PutNumber("Turret/Hold Position (turns)", _holdPosition.value());
#endif
}

void Turret::setShooterVelocity(units::turns_per_second_t velocity) {
    _cachedShooterTarget = velocity;
    _shooterMotor.SetControl(_shooterVelocityRequest.WithVelocity(velocity));
}

void Turret::runUptake() {
    if (_uptakeStallReversing) {
        if ((frc::Timer::GetFPGATimestamp() - _uptakeReverseStartTime) >= kUptakeReverseTime) {
            // Reverse window expired — resume normal velocity
            _uptakeStallReversing = false;
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        } else {
            // Still within reverse window — keep reversing
            _uptakeMotor.SetControl(_uptakeVoltageRequest.WithOutput(kUptakeReverseVoltage));
        }
    } else if (_cachedUptakeStatorCurrent > kUptakeStallAmps) {
        // Stall detected — begin reverse
        _uptakeStallReversing = true;
        _uptakeReverseStartTime = frc::Timer::GetFPGATimestamp();
        _uptakeMotor.SetControl(_uptakeVoltageRequest.WithOutput(kUptakeReverseVoltage));
    } else {
        // Normal operation
        _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
    }
}

void Turret::stopRotation() {
    _rotationMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stopShooter() {
    _shooterMotor.SetControl(_voltageRequest.WithOutput(0_V));
}

void Turret::stopUptake() {
    _uptakeStallReversing = false;
    _uptakeMotor.SetControl(_uptakeVoltageRequest.WithOutput(0_V));
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
    // Force position-hold to re-latch from actual motor position when manual control resumes,
    // rather than snapping back to wherever the hold was last set during auto-aim.
    _isHoldingPosition = false;
}

void Turret::setTargetAngle(units::degree_t angle) {
    // Convert to turns and forward to position control
    setTargetPosition(units::turn_t{angle});
}

void Turret::setTargetPosition(units::turn_t position) {
    _targetAngle = units::degree_t{position};
    // Clamp to soft limit window before commanding
    position = units::math::max(kMinPos, units::math::min(kMaxPos, position));

    // Detect large wrap-around moves and use half the cruise velocity to protect
    // the mechanism from high-speed cross-range sweeps.
    auto currentPos = _rotationMotor.GetPosition().GetValue();
    auto delta = units::math::abs(position - currentPos);
    auto cruiseVelocity = (delta > kLargeMoveThreshold)
        ? kRotationCruiseVelocity * 0.5
        : kRotationCruiseVelocity;

    _rotationMotor.SetControl(
        _mmPositionRequest
            .WithPosition(position)
            .WithVelocity(cruiseVelocity)
            .WithAcceleration(kRotationAcceleration)
    );
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

units::turn_t Turret::computeAimPosition(const frc::Pose2d& robotPose, 
                                          const frc::Pose2d& targetPose) const {
    // Transform target into robot frame
    auto robotToTarget = targetPose.Translation() - robotPose.Translation();
    auto robotToTargetInRobotFrame = robotToTarget.RotateBy(-robotPose.Rotation());

    // Subtract turret pivot offset
    auto turretPivot = vision::kTurretPivotInRobot;
    auto pivotToTarget = robotToTargetInRobotFrame - turretPivot;

    // atan2 gives angle from robot forward (+X).
    // Range: [-0.5_tr, +0.5_tr]  (i.e. -180° to +180°)
    // Motor calibration:
    //   0_tr     = back  (-X, atan2 = ±0.5_tr)
    //  +0.25_tr  = left  (+Y, atan2 = +0.25_tr)  [kMinPos..0 forward-left quadrant]
    //  -0.25_tr  = right (-Y, atan2 = -0.25_tr)  [now reachable with kMinPos = -0.25_tr]
    //
    // Mapping: motorPos = 0.5 - atan2_turns
    //   back  (atan2=+0.5):  0.5 - 0.5   =  0     ✓
    //   left  (atan2=+0.25): 0.5 - 0.25  = +0.25  ✓
    //   right (atan2=-0.25): 0.5 - (-0.25) = 0.75 → wraps to -0.25 ✓  (now within kMinPos)
    double atan2Turns = std::atan2(pivotToTarget.Y().value(), pivotToTarget.X().value())
                        / (2.0 * std::numbers::pi);
    units::turn_t motorPos{0.5 - atan2Turns};

    // Wrap into [kMinPos, kMinPos + 1.0_tr) so the full soft-limit range is reachable.
    // Using a fixed (-0.5, +0.5] window would silently clip the negative travel beyond -0.5_tr.
    while (motorPos > kMinPos + 1.0_tr) motorPos -= 1.0_tr;
    while (motorPos < kMinPos)          motorPos += 1.0_tr;

    // Apply jog-wheel rotation offset (positive = left, negative = right)
    motorPos += _rotationOffset;
#if BOT_TRACE_SUBSYSTEMS
    // Telemetry
    frc::SmartDashboard::PutNumber("Turret/AimAngle Raw (turns)", atan2Turns);
    frc::SmartDashboard::PutNumber("Turret/AimAngle Motor (turns)", motorPos.value());
#endif
    return motorPos;
}

// Command factories
frc2::CommandPtr Turret::aimAtTargetCommand(std::function<frc::Pose2d()> robotPoseSupplier,
                                              std::function<frc::Pose2d()> targetPoseSupplier) {
    return RunOnce([this] { enableAutoAim(); })
        .AndThen(
            Run([this, robotPoseSupplier, targetPoseSupplier] {
                auto robotPose = robotPoseSupplier();
                auto targetPose = targetPoseSupplier();
                auto aimPos = computeAimPosition(robotPose, targetPose);
                setTargetPosition(aimPos);
            })
        )
        .FinallyDo([this] { disableAutoAim(); })
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

frc2::CommandPtr Turret::stopCommand() {
    return RunOnce([this] { stop(); })
        .WithName("StopTurret");
}

units::turns_per_second_t Turret::velocityFromDistance(units::meter_t distance) const {
    constexpr units::meter_t kNearDist  = 1.75_m;
    constexpr units::meter_t kFarDist   = 5.5_m;
    constexpr double         kNearSpeed = 40.0;
    constexpr double         kFarSpeed  = 64.0;

    if (! _autoAimEnabled)
        return kFallbackShooterVelocity;

    // Scale distance before lookup. Decrease to reduce power (e.g. for balls that fly farther).
    distance *= _distanceScale;

    double t = (distance - kNearDist) / (kFarDist - kNearDist);
    t = std::clamp(t, 0.0, 1.0);
    return units::turns_per_second_t{kNearSpeed + t * (kFarSpeed - kNearSpeed)} + _shooterSpeedOffset;
}

frc2::CommandPtr Turret::shooterOnCommand(std::function<units::meter_t()> distanceFn) {
    // Resolve velocity: use distance supplier when available, else fixed constant.
    auto targetVelocity = [this, distanceFn]() -> units::turns_per_second_t {
        return distanceFn ? velocityFromDistance(distanceFn()) : kShooterVelocity;
    };

    // Warm path: already at speed → start uptake immediately.
    // Cold path: spin up, wait until ready, then start uptake.
    // Either way, the flywheel keeps tracking distance while the command runs.
    // NOTE: frc2::cmd::RunOnce/Run (free functions, no subsystem requirement) are used
    // intentionally so that these commands do NOT interrupt aimAtTargetCommand.
    auto warm = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        frc2::cmd::RunOnce([this] { runUptake(); })
    );

    auto cold = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        frc2::cmd::RunOnce([this] { runUptake(); })
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
        // NOTE: frc2::cmd::RunOnce/Run (free functions, no subsystem requirement) are used
        // intentionally so that this command does NOT interrupt aimAtTargetCommand.
        frc2::cmd::RunOnce([this, distanceFn] { setShooterVelocity(velocityFromDistance(distanceFn())); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        // Once ready: continuously track distance and run uptake
        frc2::cmd::Run([this, distanceFn] {
            setShooterVelocity(velocityFromDistance(distanceFn()));
            runUptake();
        })
    )
    .FinallyDo([this] {
        stopUptake();
        stopShooter();
    })
    .WithName("ShootAtDistance");
}

frc2::CommandPtr Turret::manualUnjamCommand() {
    return frc2::cmd::StartEnd(
        [this] {
            // Override any auto-stall state and drive the uptake backwards immediately.
            _uptakeStallReversing = false;
            _uptakeMotor.SetControl(_uptakeVoltageRequest.WithOutput(kUptakeReverseVoltage));
        },
        [this] {
            stopUptake();
        }
    ).WithName("ManualUnjam");
}

frc2::CommandPtr Turret::shooterOffCommand() {
    return frc2::cmd::RunOnce([this] {
        stopUptake();
        stopShooter();
    }).WithName("ShooterOff");
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
