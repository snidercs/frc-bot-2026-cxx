// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/CommandJoystick.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "config.hpp"
#include "container.hpp"
#include "inpututil.hpp"
#include "shaker.hpp"
#include "vision.hpp"
#include "visionmulti.hpp"
#include "visionsim.hpp"

using pathplanner::AutoBuilder;

namespace indy {

class JoystickContainer : public Container {
public:
    JoystickContainer() = default;
    ~JoystickContainer() override {}

protected:
    void configureBindings() override
    {
        // std::cout << "JoystickContainer::configureBindings()" << std::endl;

        const auto deadband = config::number("drive_deadband");
        const auto exponent = config::number("drive_input_exponent");
        const auto rotExp = config::number("rotate_input_exponent");
        const auto rotDead = config::number("rotate_deadband");

        // clang-format off
        drivetrain().SetDefaultCommand (
            drivetrain().ApplyRequest ([this, deadband, exponent, rotExp, rotDead]() -> auto&& {
                return drive
                    .WithVelocityX(-applyCurve(
                        _sticks[0].GetHID().GetRawAxis(1), deadband, exponent) * MaxSpeed)
                    .WithVelocityY(-applyCurve(
                        _sticks[0].GetHID().GetRawAxis(0), deadband, exponent) * MaxSpeed)
                    .WithRotationalRate(-applyCurve(
                        _sticks[1].GetHID().GetRawAxis(0), rotDead, rotExp) * MaxAngularRate);
            }));

        _sticks[0].Button(config::integer("heading_button_index")).OnTrue (
            drivetrain().RunOnce ([this] { drivetrain().SeedFieldCentric(); }));

        // Intake control
        _sticks[0].Button(config::integer("intake_trigger_index")).WhileTrue (
            intake().intakeCommand()
                .AlongWith(shaker().spinCommand(0.1).AsProxy()));
        _sticks[1].Button(config::integer("intake_eject_index")).WhileTrue (
            intake().ejectCommand());
        
        // Shooter Control — run shaker in parallel to agitate while shooting
        _sticks[1].Button(config::integer("turret_shoot_button_index")).WhileTrue (
            turret().shootAtDistanceCommand([this] {
                return drivetrain().GetState().Pose.Translation()
                           .Distance(landmarks::hubPosition());
            }).AlongWith(shaker().spinCommand(0.3).AsProxy()));

        // Climber control
        _sticks[0].Button(config::integer("climber_climb_button_index")).WhileTrue (
            climber().climbCommand());
        _sticks[0].Button(config::integer("climber_lower_button_index")).WhileTrue (
            climber().lowerCommand());

        // Drive jitter (intake agitator) - front/back on button 1, left/right on button 2
        _sticks[0].Button(5).OnTrue(jitterCommand(false));
        _sticks[0].Button(6).OnTrue(jitterCommand(true));

        // Disable climber soft limits while held; re-enable and zero position on release
        _sticks[1].Button(4).OnTrue(climber().disableSoftLimitsCommand())
                             .OnFalse(climber().enableSoftLimitsAndResetCommand());

        // Auto-aim: toggle button 16 to track hub with turret rotation.
        // First press enables auto-aim; second press (or any interruption) disables it.
        _sticks[1].Button(16).ToggleOnTrue(
            turret().aimAtTargetCommand(
                [this] { return drivetrain().GetState().Pose; },
                [this] { return frc::Pose2d{landmarks::hubPosition(), frc::Rotation2d{}}; }
            ));

        // Zero turret rotation position
        _sticks[1].Button(3).OnTrue(turret().calibrateRotationZero());

        // Shaker: toggle stick 1 button 17 to oscillate forward/reverse
        _sticks[1].Button(17).ToggleOnTrue(shaker().oscillateCommand(0.4));

        // Manual turret rotation
        const auto rotStick = config::integer("turret_rotation_axis_stick");
        const auto rotIdx = config::integer("turret_rotation_axis_index");
        const auto rotGain = config::number("turret_roation_gain");
        turret().SetDefaultCommand(
            turret().manualRotateCommand([this, rotStick, rotIdx, rotGain] { 
                return rotGain * _sticks[rotStick].GetHID().GetRawAxis(rotIdx); // Right stick X
            }));
        
        _sticks[0].Button(4).OnTrue(
            drivetrain().RunOnce([this] {
                static constexpr units::meter_t kCornerOffset = 0.44_m;
                static constexpr units::meter_t kFieldHeight = 8.069_m;
                static constexpr units::meter_t kFieldLength = 16.535_m; // 2026 Rebuilt field

#if 0
                auto alliance = frc::DriverStation::GetAlliance();
                if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
                    drivetrain().ResetPose(
                        frc::Pose2d{kFieldLength - kCornerOffset, kCornerOffset, 180_deg});
                } else {
                    drivetrain().ResetPose(
                        frc::Pose2d{kCornerOffset, kCornerOffset, 0_deg});
                }
#else
                // Back-left corner on blue / back-right corner on red
                auto alliance = frc::DriverStation::GetAlliance();
                if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
                    drivetrain().ResetPose(
                        frc::Pose2d{kFieldLength - kCornerOffset, kFieldHeight - kCornerOffset, 180_deg});
                } else {
                    drivetrain().ResetPose(
                        frc::Pose2d{kCornerOffset, kFieldHeight - kCornerOffset, 0_deg});
                }
#endif
        }));

        // clang-format on
    }

private:
    frc2::CommandJoystick _sticks[2] = {
        frc2::CommandJoystick { 0 },
        frc2::CommandJoystick { 1 }
    };
};

class GamepadContainer : public Container {
public:
    GamepadContainer() = default;
    ~GamepadContainer() override {}

    void configureBindings() override
    {
        // std::cout << "GamepadContainer::configureBindings()" << std::endl;

        const auto deadband = config::number("drive_deadband");
        const auto exponent = config::number("drive_input_exponent");

        // clang-format off
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain().SetDefaultCommand (
            // Drivetrain will execute this command periodically
            drivetrain().ApplyRequest ([this, deadband, exponent]() -> auto&& {
                return drive
                    .WithVelocityX(-applyCurve(joystick.GetLeftY(), deadband, exponent) * MaxSpeed)       // Drive forward with negative Y (forward)
                    .WithVelocityY(-applyCurve(joystick.GetLeftX(), deadband, exponent) * MaxSpeed)       // Drive left with negative X (left)
                    .WithRotationalRate(-applyCurve(joystick.GetRightX(), deadband, exponent) * MaxAngularRate); // Drive counterclockwise with negative X (left)
            }));

        joystick.A().WhileTrue (drivetrain().ApplyRequest ([this]() -> auto&& { return brake; }));
        joystick.B().WhileTrue (drivetrain().ApplyRequest ([this]() -> auto&& {
            return point.WithModuleDirection (frc::Rotation2d { -joystick.GetLeftY(), -joystick.GetLeftX() });
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        (joystick.Back() && joystick.Y()).WhileTrue (drivetrain().SysIdDynamic (frc2::sysid::Direction::kForward));
        (joystick.Back() && joystick.X()).WhileTrue (drivetrain().SysIdDynamic (frc2::sysid::Direction::kReverse));
        (joystick.Start() && joystick.Y()).WhileTrue (drivetrain().SysIdQuasistatic (frc2::sysid::Direction::kForward));
        (joystick.Start() && joystick.X()).WhileTrue (drivetrain().SysIdQuasistatic (frc2::sysid::Direction::kReverse));

        // reset the field-centric heading on left bumper press
        joystick.LeftBumper().OnTrue (drivetrain().RunOnce ([this] { drivetrain().SeedFieldCentric(); }));

        // Intake controls (use RightBumper/RightTrigger to avoid conflict with A/B buttons)
        joystick.RightBumper().WhileTrue (intake().intakeCommand());
        joystick.RightTrigger().WhileTrue (intake().ejectCommand());
        
        // Climber controls
        joystick.Button(config::integer("climber_climb_button_index")).WhileTrue (climber().climbCommand());
        joystick.Button(config::integer("climber_lower_button_index")).WhileTrue (climber().lowerCommand());

        // clang-format on
    }

private:
    frc2::CommandXboxController joystick { 0 };
};

Container::Container()
{
    // Construct drivetrain with 250 Hz odometry update frequency to prevent CAN stale errors
    _drivetrain = std::make_unique<indy::CommandSwerveDrivetrain> (
        TunerConstants::DrivetrainConstants,
        250_Hz,
        TunerConstants::FrontLeft,
        TunerConstants::FrontRight,
        TunerConstants::BackLeft,
        TunerConstants::BackRight);

    _intake = std::make_unique<indy::Intake>();
    _climber = std::make_unique<indy::Climber>();
    _turret = std::make_unique<indy::Turret>();
    _shaker = std::make_unique<indy::Shaker>();

#if BOT_VISION
    if (frc::RobotBase::IsSimulation()) {
        _vision = std::make_unique<VisionSim>();
    } else {
        _vision = std::make_unique<VisionMulti>();
    }
#endif

    
    // Configure PathPlanner AutoBuilder for autonomous
    bool pathPlannerConfigured = true;
    try {
        drivetrain().ConfigurePathPlanner();
        std::cout << "Successfully configured PathPlanner AutoBuilder" << std::endl;
    } catch (const std::exception& e) {
        pathPlannerConfigured = false;
        std::cerr << "Failed to configure PathPlanner: " << e.what() << std::endl;
    }

    if (pathPlannerConfigured) {
        try {
            using nc = pathplanner::NamedCommands;
            nc::registerCommand("shooterOn", turret().shooterOnCommand([this] {
                return drivetrain().GetState().Pose.Translation()
                           .Distance(landmarks::hubPosition());
            }));
            nc::registerCommand("shooterOff",  turret().shooterOffCommand());
            nc::registerCommand("turretAim", turret().aimAtTargetCommand(
                [this] { return drivetrain().GetState().Pose; },
                [this] { return frc::Pose2d{landmarks::hubPosition(), frc::Rotation2d{}}; }
            ));
            
            nc::registerCommand("turretStop",  turret().stopCommand());
            nc::registerCommand("intakeStart", intake().startCommand());
            nc::registerCommand("intakeStutter", intake().stutterCommand());
            nc::registerCommand("intakeStop",  intake().stopCommand());
            nc::registerCommand("driveJitter", jitterCommand());

            _autoBuilder = AutoBuilder::buildAutoChooser (config::str ("auto_default_name"));
            frc::SmartDashboard::PutData ("AutoChooser", &_autoBuilder.value());
        } catch (const std::exception& e) {
            pathPlannerConfigured = false;
            std::cerr << "Autobuilder failed: " << e.what() << std::endl;
        }
    }
}

frc2::CommandPtr Container::jitterCommand(bool leftToRight)
{
    // Move ±2 inches (0.0508 m) at 0.3 m/s → each leg takes ~0.17 s
    static constexpr units::meters_per_second_t kJitterSpeed = 0.3_mps;
    static constexpr units::second_t kJitterDuration = 0.17_s;

    auto neg = [this, leftToRight] { return leftToRight
        ? robotCentric.WithVelocityY(-kJitterSpeed)
        : robotCentric.WithVelocityX(-kJitterSpeed); };
    auto pos = [this, leftToRight] { return leftToRight
        ? robotCentric.WithVelocityY( kJitterSpeed)
        : robotCentric.WithVelocityX( kJitterSpeed); };

    return frc2::cmd::Sequence(
               drivetrain().ApplyRequest(neg).WithTimeout(kJitterDuration),
               drivetrain().ApplyRequest(pos).WithTimeout(kJitterDuration),
               drivetrain().ApplyRequest(neg).WithTimeout(kJitterDuration),
               drivetrain().ApplyRequest(pos).WithTimeout(kJitterDuration),
               drivetrain().ApplyRequest([] { return swerve::requests::SwerveDriveBrake{}; }).WithTimeout(0.1_s)
           )
        .WithName("DriveJitter");
}

Container::~Container()
{
    _vision.reset();
    _turret.reset();
    _climber.reset();
    _intake.reset();
    _drivetrain.reset();
}

std::unique_ptr<Container> Container::create()
{
    std::unique_ptr<Container> rc;
    if (config::boolean ("gamepad"))
        rc.reset (new GamepadContainer());
    else
        rc.reset (new JoystickContainer());

    rc->configureBindingsInternal();
    return rc;
}

void Container::configureBindingsInternal()
{
    // Configure subclass bindings.
    configureBindings();

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue (
        drivetrain().ApplyRequest ([] {
                        return swerve::requests::Idle {};
                    })
            .IgnoringDisable (true));

    drivetrain().RegisterTelemetry ([this] (auto const& state) { 
        logger.Telemeterize (state);
    });
}

frc2::CommandPtr Container::GetAutonomousCommand()
{
    std::string autoName = config::str("auto_default_name");
    if (_autoBuilder.has_value()) {
        if (auto* selected = _autoBuilder.value().GetSelected(); selected != nullptr)
            autoName = selected->GetName();
    }

    try {
        return pathplanner::PathPlannerAuto(autoName).ToPtr();
    } catch (const std::exception& e) {
        std::cerr << "Failed to load PathPlanner auto '" << autoName << "': " << e.what() << std::endl;
        return frc2::cmd::None();
    }
}

}
