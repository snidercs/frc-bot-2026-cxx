// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "drivetrain.hpp"
#include "intake.hpp"
#include "climber.hpp"
#include "telemetry.hpp"

class Container {
public:
    Container();
    virtual ~Container();

    static std::unique_ptr<Container> create();

    auto& drivetrain() noexcept { return *_drivetrain; }
    auto& intake() noexcept { return *_intake; }
    auto& climber() noexcept { return *_climber; }
    frc2::CommandPtr GetAutonomousCommand();

protected:
    /** Override to configure controller bindings. */
    virtual void configureBindings() = 0;

    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps;                 // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric {}
                                               .WithDeadband (MaxSpeed * 0.1)
                                               .WithRotationalDeadband (MaxAngularRate * 0.1)                     // Add a 10% deadband
                                               .WithDriveRequestType (swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake {};
    swerve::requests::PointWheelsAt point {};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger { MaxSpeed };

private:
    void configureBindingsInternal();
    std::unique_ptr<subsystems::CommandSwerveDrivetrain> _drivetrain;
    std::unique_ptr<subsystems::Intake> _intake;
    std::unique_ptr<subsystems::Climber> _climber;
};
