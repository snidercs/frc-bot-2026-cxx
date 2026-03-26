#include "turret.hpp"
#include "vision.hpp"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <numbers>
#include <cmath>

using namespace ctre::phoenix6;

namespace indy {

Turret::Turret() {
    SetName("Turret");
    configureMotors();
    frc::SmartDashboard::SetDefaultNumber("Turret/Distance Scale", _distanceScale);
    frc::SmartDashboard::SetDefaultNumber("SOTM/Lookahead (s)", _sotmLookahead);
    frc::SmartDashboard::SetDefaultNumber("SOTM/FF Gain", _sotmFFGain);
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
            ::configs::FeedbackConfigs{}
                .WithSensorToMechanismRatio(10.0)
        )
        .WithSoftwareLimitSwitch(
            ::configs::SoftwareLimitSwitchConfigs{}
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

    // Read distance scale from Elastic slider (clamped for safety)
    _distanceScale = std::clamp(
        frc::SmartDashboard::GetNumber("Turret/Distance Scale", 1.0), 0.5, 1.5);

    // Read SOTM tuning sliders (clamped for safety)
    _sotmLookahead = std::clamp(
        frc::SmartDashboard::GetNumber("SOTM/Lookahead (s)", kDefaultLookaheadTime.value()),
        0.05, 0.6);
    _sotmFFGain = std::clamp(
        frc::SmartDashboard::GetNumber("SOTM/FF Gain", kDefaultTurretFFGain),
        0.0, 2.0);

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
    constexpr units::turn_t kMinPos = -0.05542_tr;
    constexpr units::turn_t kMaxPos =  0.25_tr;
    position = units::math::max(kMinPos, units::math::min(kMaxPos, position));
    _rotationMotor.SetControl(_positionRequest.WithPosition(position));
}

void Turret::setTargetPositionWithFF(units::turn_t position,
                                     units::turns_per_second_t velocityFF) {
    _targetAngle = units::degree_t{position};
    constexpr units::turn_t kMinPos = -0.05542_tr;
    constexpr units::turn_t kMaxPos =  0.25_tr;
    position = units::math::max(kMinPos, units::math::min(kMaxPos, position));
    // Clamp velocity feedforward for safety
    velocityFF = units::math::max(-kMaxTurretFF, units::math::min(kMaxTurretFF, velocityFF));
    _rotationMotor.SetControl(
        _positionRequest.WithPosition(position).WithVelocity(velocityFF));
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
    //   0_tr    = back  (-X, atan2 = ±0.5_tr)
    //  +0.25_tr = left  (+Y, atan2 = +0.25_tr)
    //  -0.05_tr = right (-Y, atan2 = -0.25_tr, clamped)
    //
    // Mapping: motorPos = 0.5_tr - |atan2_turns| ... no.
    // Direct: atan2_turns(back) = ±0.5, we want 0.
    //         atan2_turns(left) = +0.25, we want +0.25.
    // So: motorPos = atan2_turns - 0.5  (then wrap and clamp)
    //   back:  0.5 - 0.5 =  0    ✓
    //   left: 0.25 - 0.5 = -0.25 ✗  (want +0.25)
    //
    // Try: motorPos = 0.5 - atan2_turns
    //   back (atan2=+0.5):  0.5 - 0.5  =  0    ✓
    //   left (atan2=+0.25): 0.5 - 0.25 = +0.25 ✓
    //   right(atan2=-0.25): 0.5 - (-0.25) = 0.75 → wraps to -0.25 (outside limit, clamped) ✓
    double atan2Turns = std::atan2(pivotToTarget.Y().value(), pivotToTarget.X().value())
                        / (2.0 * std::numbers::pi);
    units::turn_t motorPos{0.5 - atan2Turns};

    // Wrap into (-0.5, +0.5]
    while (motorPos >  0.5_tr) motorPos -= 1.0_tr;
    while (motorPos < -0.5_tr) motorPos += 1.0_tr;
#if BOT_TRACE_SUBSYSTEMS
    // Telemetry
    frc::SmartDashboard::PutNumber("Turret/AimAngle Raw (turns)", atan2Turns);
    frc::SmartDashboard::PutNumber("Turret/AimAngle Motor (turns)", motorPos.value());
#endif
    return motorPos;
}

std::pair<units::turn_t, units::turns_per_second_t>
Turret::computeAimWithSOTM(const frc::Pose2d& robotPose,
                           const frc::Pose2d& targetPose,
                           const frc::ChassisSpeeds& fieldSpeeds) const {
    // --- 1. Pose prediction: where will the robot be when the ball arrives? ---
    units::second_t lookahead{_sotmLookahead};
    frc::Pose2d predictedPose{
        robotPose.X() + fieldSpeeds.vx * lookahead,
        robotPose.Y() + fieldSpeeds.vy * lookahead,
        robotPose.Rotation() + frc::Rotation2d{fieldSpeeds.omega * lookahead}};

    // Compute aim from predicted pose (reuse existing math)
    auto aimPos = computeAimPosition(predictedPose, targetPose);

    // --- 2. Turret velocity feedforward ---
    // The turret must counter-rotate against robot yaw AND track the tangential
    // component of the robot's motion relative to the target.
    //
    // Formula (in rad/s, then converted to turret turns/s):
    //   turretAngVel = -(omega + v_tangential / distance)
    //
    // omega           = robot yaw rate (rad/s, CCW positive)
    // v_tangential    = component of field velocity perpendicular to robot→target line
    // distance        = robot→target distance

    auto robotToTarget = targetPose.Translation() - robotPose.Translation();
    auto distance = robotToTarget.Norm();

    units::turns_per_second_t turretFF{0};
    if (distance > 0.5_m) {
        // Unit vector from robot to target
        double dx = robotToTarget.X().value();
        double dy = robotToTarget.Y().value();
        double dist = distance.value();
        double ux = dx / dist;
        double uy = dy / dist;

        // Tangential velocity: cross product of unit vector × velocity gives the
        // perpendicular component (positive = target moving CCW from robot's POV)
        double vx = fieldSpeeds.vx.value();
        double vy = fieldSpeeds.vy.value();
        double vTangential = -(ux * vy - uy * vx);  // negative because turret frame is mirrored

        double omegaRad = fieldSpeeds.omega.value();  // rad/s

        // Total angular rate the turret needs to track (rad/s)
        double turretAngVelRad = -(omegaRad + vTangential / dist);

        // Convert rad/s → turret turns/s (1 turn = 2π rad)
        double turretAngVelTps = turretAngVelRad / (2.0 * std::numbers::pi);

        turretFF = units::turns_per_second_t{turretAngVelTps * _sotmFFGain};
    }

#if BOT_TRACE_SUBSYSTEMS
    frc::SmartDashboard::PutNumber("SOTM/Aim Position (tr)", aimPos.value());
    frc::SmartDashboard::PutNumber("SOTM/Turret FF (tps)", turretFF.value());
    frc::SmartDashboard::PutNumber("SOTM/Distance (m)", distance.value());
    frc::SmartDashboard::PutNumber("SOTM/Predicted X", predictedPose.X().value());
    frc::SmartDashboard::PutNumber("SOTM/Predicted Y", predictedPose.Y().value());
#endif

    return {aimPos, turretFF};
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

// --- SOTM command factories ---

frc2::CommandPtr Turret::sotmAimCommand(std::function<frc::Pose2d()> robotPoseSupplier,
                                        std::function<frc::Pose2d()> targetPoseSupplier,
                                        std::function<frc::ChassisSpeeds()> speedsSupplier) {
    return RunOnce([this] { enableAutoAim(); })
        .AndThen(
            Run([this, robotPoseSupplier, targetPoseSupplier, speedsSupplier] {
                auto robotPose = robotPoseSupplier();
                auto targetPose = targetPoseSupplier();
                auto robotSpeeds = speedsSupplier();

                // Convert robot-relative speeds to field-relative for SOTM math
                auto fieldSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
                    robotSpeeds, robotPose.Rotation());

                // Check speed gate: only apply SOTM when moving at a reasonable speed
                auto linearSpeed = units::math::sqrt(
                    fieldSpeeds.vx * fieldSpeeds.vx + fieldSpeeds.vy * fieldSpeeds.vy);

                if (linearSpeed < kMaxSOTMSpeed) {
                    // SOTM path: predicted aim + velocity feedforward
                    auto [aimPos, turretFF] = computeAimWithSOTM(
                        robotPose, targetPose, fieldSpeeds);
                    setTargetPositionWithFF(aimPos, turretFF);
                } else {
                    // Speed too high — fall back to static aim (no prediction, no FF)
                    auto aimPos = computeAimPosition(robotPose, targetPose);
                    setTargetPosition(aimPos);
                }
            })
        )
        .FinallyDo([this] { disableAutoAim(); })
        .WithName("SOTMAim");
}

frc2::CommandPtr Turret::sotmShootCommand(std::function<units::meter_t()> distanceFn,
                                          std::function<units::meters_per_second_t()> radialVelFn) {
    return frc2::cmd::Sequence(
        // Spin up with initial SOTM-compensated speed
        frc2::cmd::RunOnce([this, distanceFn, radialVelFn] {
            auto effectiveDist = distanceFn() + radialVelFn() * units::second_t{_sotmLookahead};
            setShooterVelocity(velocityFromDistance(effectiveDist));
        }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        // Once ready: continuously track SOTM-compensated distance and run uptake
        frc2::cmd::Run([this, distanceFn, radialVelFn] {
            auto effectiveDist = distanceFn() + radialVelFn() * units::second_t{_sotmLookahead};
            setShooterVelocity(velocityFromDistance(effectiveDist));
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
#if BOT_TRACE_SUBSYSTEMS
            frc::SmartDashboard::PutNumber("SOTM/Effective Distance (m)", effectiveDist.value());
            frc::SmartDashboard::PutNumber("SOTM/Radial Vel (m/s)", radialVelFn().value());
#endif
        })
    )
    .FinallyDo([this] {
        stopUptake();
        stopShooter();
    })
    .WithName("SOTMShoot");
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
    constexpr units::meter_t kNearDist  = 1.75_m;
    constexpr units::meter_t kFarDist   = 5.5_m;
    constexpr double         kNearSpeed = 46.0;
    constexpr double         kFarSpeed  = 65.2;

    if (! _autoAimEnabled)
        return kFallbackShooterVelocity;

    // Scale distance before lookup. Decrease to reduce power (e.g. for balls that fly farther).
    distance *= _distanceScale;

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
    // NOTE: frc2::cmd::RunOnce/Run (free functions, no subsystem requirement) are used
    // intentionally so that these commands do NOT interrupt aimAtTargetCommand.
    auto warm = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        frc2::cmd::RunOnce([this] {
            _uptakeMotor.SetControl(_uptakeVelocityRequest.WithVelocity(kUptakeVelocity));
        })
    );

    auto cold = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this, targetVelocity] { setShooterVelocity(targetVelocity()); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        frc2::cmd::RunOnce([this] {
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
        // NOTE: frc2::cmd::RunOnce/Run (free functions, no subsystem requirement) are used
        // intentionally so that this command does NOT interrupt aimAtTargetCommand.
        frc2::cmd::RunOnce([this, distanceFn] { setShooterVelocity(velocityFromDistance(distanceFn())); }),
        frc2::cmd::WaitUntil([this] { return isShooterReady(); }),
        // Once ready: continuously track distance and run uptake
        frc2::cmd::Run([this, distanceFn] {
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
