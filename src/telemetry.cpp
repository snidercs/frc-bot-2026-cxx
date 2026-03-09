#include "telemetry.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

#define BOT_SIGNAL_LOGGER 0

using namespace ctre::phoenix6;

Telemetry::Telemetry(units::meters_per_second_t maxSpeed) : MaxSpeed{maxSpeed}
{
#if BOT_SIGNAL_LOGGER
    SignalLogger::Start();
#endif

    /* Set up the module state Mechanism2d telemetry */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    /* Telemeterize the swerve drive state — decompose structs into primitives */
    tkit::RecordOutput("DriveState/Pose/x",       state.Pose.X().value());
    tkit::RecordOutput("DriveState/Pose/y",       state.Pose.Y().value());
    tkit::RecordOutput("DriveState/Pose/degrees", state.Pose.Rotation().Degrees().value());

    tkit::RecordOutput("DriveState/Speeds/vx",    state.Speeds.vx.value());
    tkit::RecordOutput("DriveState/Speeds/vy",    state.Speeds.vy.value());
    tkit::RecordOutput("DriveState/Speeds/omega", state.Speeds.omega.value());

    for (size_t i = 0; i < state.ModuleStates.size(); ++i) {
        std::string base = "DriveState/ModuleStates/" + std::to_string(i);
        tkit::RecordOutput(base + "/speed",  state.ModuleStates[i].speed.value());
        tkit::RecordOutput(base + "/angle",  state.ModuleStates[i].angle.Degrees().value());
    }

    for (size_t i = 0; i < state.ModuleTargets.size(); ++i) {
        std::string base = "DriveState/ModuleTargets/" + std::to_string(i);
        tkit::RecordOutput(base + "/speed", state.ModuleTargets[i].speed.value());
        tkit::RecordOutput(base + "/angle", state.ModuleTargets[i].angle.Degrees().value());
    }

    for (size_t i = 0; i < state.ModulePositions.size(); ++i) {
        std::string base = "DriveState/ModulePositions/" + std::to_string(i);
        tkit::RecordOutput(base + "/distance", state.ModulePositions[i].distance.value());
        tkit::RecordOutput(base + "/angle",    state.ModulePositions[i].angle.Degrees().value());
    }

    tkit::RecordOutput("DriveState/Timestamp",        state.Timestamp.value());
    tkit::RecordOutput("DriveState/OdometryFrequency", 1.0 / state.OdometryPeriod.value());

    /* Telemeterize the pose to a Field2d */
    tkit::RecordOutput("Pose/robotPose/x",       state.Pose.X().value());
    tkit::RecordOutput("Pose/robotPose/y",       state.Pose.Y().value());
    tkit::RecordOutput("Pose/robotPose/degrees", state.Pose.Rotation().Degrees().value());

    /* Telemeterize each module state to a Mechanism2d */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed / (2 * MaxSpeed));
    }
}
