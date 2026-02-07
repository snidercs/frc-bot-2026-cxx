// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/CommandJoystick.h>

#include "config.hpp"
#include "container.hpp"

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
                    .WithVelocityX(-_sticks[1].GetHID().GetRawAxis(1) * MaxSpeed)
                    .WithVelocityY(-_sticks[1].GetHID().GetRawAxis(0) * MaxSpeed)
                    .WithRotationalRate (-_sticks[0].GetHID().GetRawAxis(0) * MaxAngularRate);
            }));

        _sticks[0].Button(config::integer("heading_button_index")).OnTrue (
            drivetrain().RunOnce ([this] { drivetrain().SeedFieldCentric(); }));

        // Intake control
        _sticks[1].Button(config::integer("intake_trigger_index")).WhileTrue (
            intake().intakeCommand());
        
        // Climber control
        _sticks[0].Button(config::integer("climber_climb_button_index")).WhileTrue (
            climber().climbCommand());
        _sticks[0].Button(config::integer("climber_lower_button_index")).WhileTrue (
            climber().lowerCommand());
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

        // Intake controls
        joystick.Button(1).WhileTrue (intake().intakeCommand());
        joystick.Button(2).WhileTrue (intake().ejectCommand());
        
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
    _drivetrain = std::make_unique<subsystems::CommandSwerveDrivetrain>(
        TunerConstants::DrivetrainConstants,
        250_Hz,
        TunerConstants::FrontLeft,
        TunerConstants::FrontRight,
        TunerConstants::BackLeft,
        TunerConstants::BackRight
    );
    
    _intake = std::make_unique<subsystems::Intake>();
    _climber = std::make_unique<subsystems::Climber>();
}

Container::~Container()
{
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
    // Store the starting X position (will be set when command starts)
    auto startX = std::make_shared<units::meter_t>(0_m);
    
    // Drive forward 3 meters from starting position, then brake
    return drivetrain().RunOnce([this, startX]() {
            // Capture the starting X position when autonomous begins
            *startX = drivetrain().GetState().Pose.X();
        })
        .AndThen(
            drivetrain().ApplyRequest([this]() -> auto&& {
                return autoForward; 
            })
            .Until([this, startX]() {
                // Stop when we've traveled 3 meters from start position
                auto currentX = drivetrain().GetState().Pose.X();
                auto distanceTraveled = units::math::abs(currentX - *startX);
                return distanceTraveled >= 1_m;
            })
        )
        .AndThen(
            drivetrain().ApplyRequest([this]() -> auto&& {
                return brake;
            })
        )
        .WithName("DriveForward3Meters");
}
