// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "config.hpp"
#include "robot.hpp"
#include "scripting.hpp"

using frc::DriverStation;
using frc::SmartDashboard;

namespace detail {
    /** Display engine and bot information banner. */
    static void displayBanner() {
   
    lua::printVersion();
    std::cout << "Engine running at "
              << config::num<int> ("period")
              << " ms" << std::endl;
    std::clog.flush();
    std::cout.flush();
    std::cerr.flush();
}
}

Robot::Robot()
{
    DriverStation::SilenceJoystickConnectionWarning (true);
    _container = Container::create();
}

void Robot::RobotInit() {
    // SmartDashboard::PutString ("Controller", 
    //     config::boolean("gamepad") ? "Gamepad" : "Flightsticks");
    config::display();
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
/** This is not ideal, but frc::StartRobot instantiates a singleton version
    of Robot main with no explicit shutdown.  Our lua engine must exist before
    and after the robot's ctor and dtor. Having lifecylce at the global scope
    helps avoid crashes when the app exits.
*/
static lua::Lifecycle sLuaEngine;

int main()
{
    if (! lua::bootstrap())
        throw std::runtime_error ("lua engine could not be bootstrapped");
    detail::displayBanner();
    return frc::StartRobot<Robot>();
}
#endif
