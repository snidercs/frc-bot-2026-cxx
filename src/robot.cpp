// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "container.hpp"
#include "frc/RobotBase.h"
#include "frc/TimedRobot.h"
#include <filesystem>
#include <iomanip>
#include <iostream>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/HALBase.h>

#include <telemetrykit/TelemetryKit.h>
#include <cameraserver/CameraServer.h>
#if BOT_DUMB_CAMERA
    #include <opencv2/core/core.hpp>
    #include <opencv2/core/types.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#endif

#include "config.hpp"
#include "mathutil.hpp"
#include "robot.hpp"
#include "scripting.hpp"
#include "turret.hpp"
#include "vision.hpp"
#include "visionsim.hpp"

#include "luabot/luabot2.hpp"

using frc::DriverStation;
using frc::SmartDashboard;
using indy::Container;

namespace detail {
/** Display engine and bot information banner. */
static void displayBanner()
{
    lua::printVersion();
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
    : frc::TimedRobot (units::millisecond_t (
          1000.0 / config::number ("period")))
{
    DriverStation::SilenceJoystickConnectionWarning (true);

    auto& logger = tkit::Logger::GetInstance();

    tkit::RecordOutput ("Metadata/ProjectName", std::string ("frc-bot-2026-cxx"));
    tkit::RecordOutput ("Metadata/ControllerType",
                        std::string (config::boolean ("gamepad") ? "Gamepad" : "Flightsticks"));

    try {
        if (frc::RobotBase::IsReal()) {
            std::string base;

            if (std::filesystem::is_directory ("/media/sda1"))
                base = "/media/sda1";
            else if (std::filesystem::is_directory ("/run/media/sda1"))
                base = "/run/media/sda1";
            else
                base = "/home/lvuser";

            std::string logPath = base + "/logs";
            std::filesystem::create_directories (logPath);
            logger.AddReceiver (std::make_unique<tkit::WPILogWriter> (logPath));
            logger.AddReceiver (std::make_unique<tkit::NetworkTablesReceiver>());
        } else {
            logger.AddReceiver (std::make_unique<tkit::NetworkTablesReceiver>());
        }

        logger.Start();
    } catch (const std::exception& e) {
        std::cerr << "[bot] could not start TK logger: " << e.what() << std::endl;
    }
    _container = Container::create();
}

void Robot::RobotInit()
{
    frc::RobotController::SetBrownoutVoltage (6.0_V);
    SmartDashboard::PutString ("Controller",
                               config::boolean ("gamepad") ? "Gamepad" : "Flightsticks");
    detail::displayPaths();
    config::display();

#if 1
    std::thread vision (cameraThread);
    vision.detach();
#endif

    auto periodMs = units::millisecond_t (GetPeriod()).value();
    std::cout << "[bot] running at " << config::number ("period") << " fps"
              << " (" << std::fixed << std::setprecision (3) << periodMs << " ms)" << std::endl;
}

void Robot::RobotPeriodic()
{
    auto& drive (_container->drivetrain());
    auto& vision (_container->vision());

#if BOT_VISION
    if (frc::RobotBase::IsReal() && (IsTeleop() || IsTest())) {
        // Field Boundary Clamping.
        // Must run BEFORE the currentPose snapshot so the residual gate in
        // processResults() sees the corrected pose, not the drifted one.
        // Only fires when the pose is clearly wrong (> kClampMargin outside
        // the physical field boundary), not on normal sensor noise.
        // Read pose once — reused for both field clamping and residual gating.
        // ResetPose() may mutate odometry state, so a second GetState().Pose
        // call after clamping could return a different value.
        auto currentPose = drive.GetState().Pose;
        auto speeds = drive.GetState().Speeds;
        
        {
            static constexpr units::meter_t kFieldLength = 16.535_m;
            static constexpr units::meter_t kFieldWidth = 8.069_m;
            static constexpr units::meter_t kClampMargin = 0.75_m;

            if (indy::math::isPoseOutOfBounds (currentPose, kFieldLength, kFieldWidth, kClampMargin)) {
                currentPose = indy::math::clampPoseToField (currentPose, kFieldLength, kFieldWidth);
                drive.ResetPose (currentPose);
    #if BOT_TRACE_VISION
                frc::SmartDashboard::PutBoolean ("Vision/PoseClamped", true);
    #endif
            } else {
    #if BOT_TRACE_VISION

                frc::SmartDashboard::PutBoolean ("Vision/PoseClamped", false);
    #endif
            }
        }

        // Poll all cameras, apply all gates (latency, tag count, distance,
        // field bounds, odometry residual), and fuse the single best candidate.
        vision.process (currentPose, speeds);

        for (const auto& measurement : vision.measurements()) {
            drive.AddVisionMeasurement (
                measurement.pose,
                measurement.timestamp,
                measurement.stdDevs);
        }
    }
#endif

#if BOT_TRACE_VISION
    // Estimated distance from fused robot pose to the hub
    auto robotPose = _container->drivetrain().GetState().Pose;
    units::meter_t distanceToHub = robotPose.Translation().Distance (indy::landmarks::hubPosition());
    tkit::RecordOutput ("Robot/DistanceToHub", distanceToHub.value());
#endif

    tkit::Logger::GetInstance().Periodic();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
    _autoCommand = _container->GetAutonomousCommand();

    if (_autoCommand) {
        frc2::CommandScheduler::GetInstance().Schedule (*_autoCommand);
    } else {
        std::cerr << "[bot] AutonomousInit: No autonomous command returned!" << std::endl;
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

    // Re-arm the vision pose reset so the first valid tag seen in teleop
    // will snap the robot pose before normal odometry takes over.
#if BOT_VISION
    _poseReset.reset();
#endif

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

void Robot::SimulationPeriodic()
{
    // Drive the shooter flywheel sim state so velocity tracks the setpoint.
    // Without this, GetVelocity() always returns 0 in sim and isShooterReady()
    // never becomes true, leaving shootAtDistanceCommand stuck in WaitUntil.
    auto& shooterSim = _container->turret().shooterMotor().GetSimState();
    shooterSim.SetSupplyVoltage (frc::RobotController::GetBatteryVoltage());
    shooterSim.SetRotorVelocity (_container->turret().cachedShooterTarget());

#if BOT_VISION
    // Advance the vision simulation with the current ground-truth drivetrain pose.
    if (auto* visionSim = dynamic_cast<indy::VisionSim*> (&_container->vision())) {
        visionSim->update (_container->drivetrain().GetState().Pose);
    }
#endif
}

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
    CS_Status status { 0 };
    if (cs::EnumerateUsbCameras (&status).empty()) {
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
    _Exit (result);
}
    #endif
#endif

#include <luabot/apriltag.ipp>
#include <luabot/frc.ipp>
#include <luabot/math.ipp>
