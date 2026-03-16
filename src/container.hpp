// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Timer.h>
#include <optional>
#include "drivetrain.hpp"
#include "intake.hpp"
#include "climber.hpp"
#include "turret.hpp"
#include "shaker.hpp"
#include "horizontalshaker.hpp"
#include "vision.hpp"
#include "telemetry.hpp"

namespace indy {

class Container {
public:
    Container();
    virtual ~Container();

    static std::unique_ptr<Container> create();

    auto& drivetrain() noexcept { return *_drivetrain; }
    auto& intake() noexcept { return *_intake; }
    auto& climber() noexcept { return *_climber; }
    auto& turret() noexcept { return *_turret; }
    auto& shaker() noexcept { return *_shaker; }
    auto& horizontalShaker() noexcept { return *_horizontalShaker; }
    auto& vision() noexcept { return *_vision; }
    frc2::CommandPtr GetAutonomousCommand();

    /** Wiggles the robot back and forth (±2 in) to help seat game pieces in the intake.
     *
     *  @param leftToRight If true, jitters left/right (Y axis); otherwise front/back (X axis).
     */
    frc2::CommandPtr jitterCommand(bool leftToRight = false);

protected:
    /** Override to configure controller bindings. */
    virtual void configureBindings() = 0;

    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps;                 // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric {}
                                               .WithDriveRequestType (swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::RobotCentric robotCentric = swerve::requests::RobotCentric {}
                                               .WithDriveRequestType (swerve::DriveRequestType::OpenLoopVoltage);
    swerve::requests::SwerveDriveBrake brake {};
    swerve::requests::PointWheelsAt point {};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger { TunerConstants::kSpeedAt12Volts };

private:
    void configureBindingsInternal();
    std::unique_ptr<indy::CommandSwerveDrivetrain> _drivetrain;
    std::unique_ptr<indy::Intake> _intake;
    std::unique_ptr<indy::Climber> _climber;
    std::unique_ptr<indy::Turret> _turret;
    std::unique_ptr<indy::Shaker> _shaker;
    std::unique_ptr<indy::HorizontalShaker> _horizontalShaker;
    std::unique_ptr<vision::VisionIO> _vision;
    std::optional<frc::SendableChooser<frc2::Command *>> _autoBuilder;
};

}
