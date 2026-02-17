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
    // Static PID controllers for trajectory following
    // These persist across calls to maintain state (integral accumulation, etc.)
    static frc::PIDController xController{10.0, 0.0, 0.0};
    static frc::PIDController yController{10.0, 0.0, 0.0};
    static frc::PIDController headingController{7.5, 0.0, 0.0};
    
    // Configure heading controller for continuous input (wraps at -π to π)
    static bool controllersInitialized = false;
    if (!controllersInitialized) {
        headingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
        controllersInitialized = true;
    }

    // Get current robot pose from odometry
    auto currentPose = GetState().Pose;
    
    // Calculate PID corrections for position error
    // sample.x/y/heading are already units, extract .value() for both measurement and setpoint
    double xCorrection = xController.Calculate(currentPose.X().value(), sample.x.value());
    double yCorrection = yController.Calculate(currentPose.Y().value(), sample.y.value());
    
    // Calculate heading correction
    double headingCorrection = headingController.Calculate(
        currentPose.Rotation().Radians().value(), 
        sample.heading.value()
    );
    
    // Combine feedforward velocities from trajectory with feedback corrections
    // sample.vx/vy/omega are already units, add correction as units
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        sample.vx + units::meters_per_second_t{xCorrection},
        sample.vy + units::meters_per_second_t{yCorrection},
        sample.omega + units::radians_per_second_t{headingCorrection},
        currentPose.Rotation()
    );
    
    // Apply the calculated chassis speeds using a field-centric request
    swerve::requests::FieldCentric request{};
    SetControl(request
        .WithVelocityX(speeds.vx)
        .WithVelocityY(speeds.vy)
        .WithRotationalRate(speeds.omega));
}
