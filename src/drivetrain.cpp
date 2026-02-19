#include "drivetrain.hpp"
#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <choreo/trajectory/SwerveSample.h>
#include <numbers>

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::FollowTrajectory(const choreo::SwerveSample& sample)
{
    // Get current robot pose from odometry
    auto currentPose = GetState().Pose;
    
    // Start with the trajectory's field-relative chassis speeds as feedforward
    auto targetSpeeds = sample.GetChassisSpeeds();
    
    // Add PID corrections for position error
    targetSpeeds.vx += units::meters_per_second_t{
        m_xController.Calculate(currentPose.X().value(), sample.x.value())};
    targetSpeeds.vy += units::meters_per_second_t{
        m_yController.Calculate(currentPose.Y().value(), sample.y.value())};
    targetSpeeds.omega += units::radians_per_second_t{
        m_headingController.Calculate(
            currentPose.Rotation().Radians().value(), sample.heading.value())};
    
    // Apply using ApplyFieldSpeeds (designed for autonomous trajectory following)
    // with module force feedforwards from the trajectory for better tracking
    SetControl(m_pathApplyFieldSpeeds
        .WithSpeeds(targetSpeeds)
        .WithWheelForceFeedforwardsX({sample.moduleForcesX.begin(), sample.moduleForcesX.end()})
        .WithWheelForceFeedforwardsY({sample.moduleForcesY.begin(), sample.moduleForcesY.end()}));
}

void CommandSwerveDrivetrain::ResetTrajectoryControllers()
{
    m_xController.Reset();
    m_yController.Reset();
    m_headingController.Reset();
}
