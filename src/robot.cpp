// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "container.hpp"
#include <filesystem>
#include <iostream>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/HALBase.h>

#include <cameraserver/CameraServer.h>
#if BOT_DUMB_CAMERA
    #include <opencv2/core/core.hpp>
    #include <opencv2/core/types.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#endif

#include "config.hpp"
#include "robot.hpp"
#include "scripting.hpp"
#include "vision.hpp"

#include "luabot/luabot2.hpp"

using frc::DriverStation;
using frc::SmartDashboard;
using bot::Container;

namespace detail {
/** Display engine and bot information banner. */
static void displayBanner()
{
    lua::printVersion();
    std::cout << "Engine running at "
              << config::num<int> ("period")
              << " ms" << std::endl;
    std::clog.flush();
    std::cout.flush();
    std::cerr.flush();
}

static void displayPaths()
{
    std::cout << "launch dir:    " << frc::filesystem::GetLaunchDirectory() << std::endl
              << "operating dir: " << frc::filesystem::GetOperatingDirectory() << std::endl;
}
} // namespace detail

Robot::Robot()
{
    DriverStation::SilenceJoystickConnectionWarning (true);
    _container = Container::create();
}

void Robot::RobotInit()
{
    frc::RobotController::SetBrownoutVoltage(6.0_V);
    SmartDashboard::PutString ("Controller",
                               config::boolean ("gamepad") ? "Gamepad" : "Flightsticks");
    detail::displayPaths();
    config::display();

#if 1
    std::thread vision (cameraThread);
    vision.detach();
#endif
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();

    // Poll all cameras and fuse measurements into the drivetrain pose estimator
    for (const auto& measurement : _container->vision().getMeasurements()) {
        _container->drivetrain().AddVisionMeasurement(
            measurement.pose,
            measurement.timestamp,
            measurement.stdDevs);
    }

    // Estimated distance from fused robot pose to the hub
    auto robotPose = _container->drivetrain().GetState().Pose;
    units::meter_t distanceToHub = robotPose.Translation().Distance(landmarks::hubPosition());
    SmartDashboard::PutNumber("Robot/DistanceToHub (m)", distanceToHub.value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
    std::cout << "AutonomousInit: Getting autonomous command..." << std::endl;
    _autoCommand = _container->GetAutonomousCommand();

    if (_autoCommand) {
        std::cout << "AutonomousInit: Scheduling autonomous command" << std::endl;
        frc2::CommandScheduler::GetInstance().Schedule(*_autoCommand);
    } else {
        std::cerr << "AutonomousInit: No autonomous command returned!" << std::endl;
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
    // Reset the optional - command is owned by scheduler
    _autoCommand.reset();
}

void Robot::TeleopInit()
{
    DriverStation::SilenceJoystickConnectionWarning (false);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running.
    if (_autoCommand) {
        _autoCommand->Cancel();
        _autoCommand.reset();
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

void Robot::cameraThread()
{
#if SIM_CAMERA_DISABLED
    if (RobotBase::IsSimulation())
        return;
#endif
    const auto cameraName = "DumbCamera";
    const auto width = 640;
    const auto height = 360;
    const auto fps = 20;

    // Only start the camera if a USB device is actually present
    CS_Status status  { 0 };
    if (cs::EnumerateUsbCameras(&status).empty()) {
        std::cout << "[camera] no USB cameras found, exiting camera thread\n";
        return;
    }

    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture (cameraName, 0);
    camera.SetExposureAuto();
    camera.SetWhiteBalanceAuto();

    // Set the resolution
    camera.SetResolution (width, height);
    camera.SetFPS (fps);

#if BOT_DUMB_CAMERA
    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo (cameraName, width, height);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true) {
        // Tell the CvSink to grab a frame from the camera and
        // put it
        // in the source mat.  If there is an error notify the
        // output.
        if (cvSink.GrabFrame (mat) == 0) {
            // Send the output the error.
            outputStream.NotifyError (cvSink.GetError());
            // skip the rest of the current iteration
            continue;
        }
    #if 1
        // Put a rectangle on the image
        // cs::rectangle (mat, cv::Point (100, 100),
        //                 cv::Point (400, 400),
        //                 cv::Scalar (255, 255, 255), 5);
        // Give the output stream a new image to display
        outputStream.PutFrame (mat);
    #endif
    }
#endif
}

#ifndef RUNNING_FRC_TESTS
    #if LUABOT_NATIVE
static lua::Lifecycle sLuaEngine;

int main()
{
    if (! lua::bootstrap())
        throw std::runtime_error ("lua engine could not be bootstrapped");
    std::filesystem::path path (frc::filesystem::GetOperatingDirectory());
    path /= "robot/robot.lua";
    auto* L = lua::state().lua_state();
    return luabot::start_robot (path.string(), L);
}

    #else

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
    int result = frc::StartRobot<Robot>();
    HAL_Shutdown();
    // Bypass C++ static destructors to avoid a crash in PathPlanner's
    // PPHolonomicDriveController destructor trying to access an already-
    // destroyed wpi::SendableRegistry mutex (static destruction order issue).
    _Exit(result);
}
    #endif
#endif

#include <luabot/apriltag.ipp>
#include <luabot/frc.ipp>
#include <luabot/math.ipp>
