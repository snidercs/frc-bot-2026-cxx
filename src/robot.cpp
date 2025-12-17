// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <lua.hpp>

#include "config.hpp"
#include "robot.hpp"

using frc::DriverStation;
using frc::SmartDashboard;

Robot::Robot()
{
    DriverStation::SilenceJoystickConnectionWarning (true);
    _container = Container::create();
}

void Robot::RobotInit() {
    SmartDashboard::PutString ("Controller", config::USE_GAMEPAD ? "Gamepad" : "Flightsticks");
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
    m_autonomousCommand = _container->GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
    DriverStation::SilenceJoystickConnectionWarning (false);
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit()
{
    DriverStation::SilenceJoystickConnectionWarning (true);
}

void Robot::TestInit()
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
