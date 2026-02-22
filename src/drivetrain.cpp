#include "drivetrain.hpp"
#include <frc/RobotController.h>
#include <frc/DriverStation.h>

using namespace subsystems;
using namespace pathplanner;

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

void CommandSwerveDrivetrain::ConfigurePathPlanner()
{
    // Load the RobotConfig from PathPlanner GUI settings
    auto config = RobotConfig::fromGUISettings();

    AutoBuilder::configure(
        [this]() { return GetState().Pose; },        // Robot pose supplier
        [this](const frc::Pose2d& pose) { ResetPose(pose); }, // Method to reset odometry
        [this]() { return GetState().Speeds; },       // ChassisSpeeds supplier (ROBOT RELATIVE)
        [this](const frc::ChassisSpeeds& speeds, const DriveFeedforwards& feedforwards) {
            SetControl(m_pathApplyRobotSpeeds
                .WithSpeeds(speeds)
                .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
        },
        std::make_shared<PPHolonomicDriveController>(
            PIDConstants { 10.0, 0.0, 0.0 },  // Translation PID constants
            PIDConstants { 7.5, 0.0, 0.0 }    // Rotation PID constants
        ),
        config,
        []() {
            // Flip paths for red alliance
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // drivetrain subsystem requirements
    );
}
