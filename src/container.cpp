// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/CommandJoystick.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "config.hpp"
#include "container.hpp"
#include "visiontest.hpp"

// enable to use single camera test command.
#define BOT_VISION_SINGLE 1

class JoystickContainer : public Container {
public:
    JoystickContainer() = default;
    ~JoystickContainer() override {}

protected:
    void configureBindings() override
    {
        // std::cout << "JoystickContainer::configureBindings()" << std::endl;

        // clang-format off
        drivetrain().SetDefaultCommand (
            drivetrain().ApplyRequest ([this]() -> auto&& {
                return drive
                    .WithVelocityX(-_sticks[0].GetHID().GetRawAxis(1) * MaxSpeed)
                    .WithVelocityY(-_sticks[0].GetHID().GetRawAxis(0) * MaxSpeed)
                    .WithRotationalRate (-_sticks[1].GetHID().GetRawAxis(0) * MaxAngularRate);
            }));

        _sticks[0].Button(config::integer("heading_button_index")).OnTrue (
            drivetrain().RunOnce ([this] { drivetrain().SeedFieldCentric(); }));

        // Intake control
        _sticks[0].Button(config::integer("intake_trigger_index")).WhileTrue (
            intake().intakeCommand());
        
        // Shooter Control
        _sticks[1].Button(config::integer("turret_shoot_button_index")).WhileTrue (
            turret().manualShootCommand());

        // Climber control

        _sticks[0].Button(config::integer("climber_climb_button_index")).WhileTrue (
            climber().climbCommand());
        _sticks[0].Button(config::integer("climber_lower_button_index")).WhileTrue (
            climber().lowerCommand());

        // Disable climber soft limits while held; re-enable and zero position on release
        _sticks[1].Button(4).OnTrue(climber().disableSoftLimitsCommand())
                             .OnFalse(climber().enableSoftLimitsAndResetCommand());

        // Zero turret rotation position
        _sticks[1].Button(3).OnTrue(turret().calibrateRotationZero());
        
        // Manual turret rotation
        const auto rotStick = config::integer("turret_rotation_axis_stick");
        const auto rotIdx = config::integer("turret_rotation_axis_index");
        const auto rotGain = config::number("turret_roation_gain");
        turret().SetDefaultCommand(
            turret().manualRotateCommand([this, rotStick, rotIdx, rotGain] { 
                return rotGain * _sticks[rotStick].GetHID().GetRawAxis(rotIdx); // Right stick X
            }));
            
#if BOT_VISION_SINGLE
        // Vision tracking test
        _sticks[0].Button(config::integer("turret_aim_button_index")).WhileTrue (
            test::createVisionTrackingTest(&turret(), &vision()));
#endif
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

        // clang-format off
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain().SetDefaultCommand (
            // Drivetrain will execute this command periodically
            drivetrain().ApplyRequest ([this]() -> auto&& {
                return drive.WithVelocityX (-joystick.GetLeftY() * MaxSpeed)      // Drive forward with negative Y (forward)
                    .WithVelocityY (-joystick.GetLeftX() * MaxSpeed)              // Drive left with negative X (left)
                    .WithRotationalRate (-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
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
        
        // Vision tracking test - left trigger
        joystick.LeftTrigger().WhileTrue (
            test::createVisionTrackingTest(&turret(), &vision()));
        // clang-format on
    }

private:
    frc2::CommandXboxController joystick { 0 };
};

Container::Container()
{
    // Construct drivetrain with 250 Hz odometry update frequency to prevent CAN stale errors
    _drivetrain = std::make_unique<subsystems::CommandSwerveDrivetrain> (
        TunerConstants::DrivetrainConstants,
        250_Hz,
        TunerConstants::FrontLeft,
        TunerConstants::FrontRight,
        TunerConstants::BackLeft,
        TunerConstants::BackRight);

    _intake = std::make_unique<subsystems::Intake>();
    _climber = std::make_unique<subsystems::Climber>();
    _turret = std::make_unique<subsystems::Turret>();
    _vision = std::make_unique<VisionIOSingle>(config::str("vision_test_camera"));

    // Register named commands for PathPlanner event markers.
    // These must be registered BEFORE creating any PathPlannerAutos.
    pathplanner::NamedCommands::registerCommand("intake", intake().intakeCommand());
    pathplanner::NamedCommands::registerCommand("eject", intake().ejectCommand());
    pathplanner::NamedCommands::registerCommand("shoot", turret().shootCommand());
    pathplanner::NamedCommands::registerCommand("spinUp", turret().spinUpCommand());
    pathplanner::NamedCommands::registerCommand("stopIntake", intake().stopCommand());
    pathplanner::NamedCommands::registerCommand("stopShooter", turret().stopCommand());
    
    // Configure PathPlanner AutoBuilder for autonomous
    try {
        drivetrain().ConfigurePathPlanner();
        std::cout << "Successfully configured PathPlanner AutoBuilder" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to configure PathPlanner: " << e.what() << std::endl;
    }
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

    drivetrain().RegisterTelemetry ([this] (auto const& state) { logger.Telemeterize (state); });
}

frc2::CommandPtr Container::GetAutonomousCommand()
{
    // Load and return a PathPlannerAuto by name.
    // The auto file must exist in src/main/deploy/pathplanner/autos/
    // and be created using the PathPlanner GUI application.
    try {
        return pathplanner::PathPlannerAuto("ShooterTest").ToPtr();
    } catch (const std::exception& e) {
        std::cerr << "Failed to load PathPlanner auto: " << e.what() << std::endl;
        return frc2::cmd::None();
    }
}
