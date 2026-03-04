#include "visionmulti.hpp"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

VisionMulti::VisionMulti()
    : _fieldLayout(vision::getFieldLayout())
{
    for (std::size_t i = 0; i < vision::kCameraNames.size(); ++i) {
        _cameras[i] = std::make_unique<CameraUnit>(
            vision::kCameraNames[i],
            vision::kRobotToCamera[i],
            _fieldLayout);
    }
}

std::vector<VisionMeasurement> VisionMulti::getMeasurements() {
    std::vector<VisionMeasurement> measurements;

    // NOTE: _fieldLayout always uses the default blue-origin (kBlueAllianceWallRightSide).
    // WPILib's pose estimator (and CTRE's AddVisionMeasurement) always expect poses in
    // the blue-origin frame regardless of alliance. Setting the origin to red would cause
    // PhotonPoseEstimator to output red-relative poses (~0–2 m) that the drivetrain
    // would interpret as being near the blue wall (~16 m), corrupting the pose estimate.

    for (auto& unit : _cameras) {
        auto results = unit->camera.GetAllUnreadResults();

        for (auto& result : results) {
            if (!result.HasTargets()) {
                _rejectedNoTargets++;
                continue;
            }

            // Gate on latency
            if (result.GetLatency() > kMaxLatency) {
                _rejectedStale++;
                continue;
            }

            // Gate on best-target ambiguity
            if (result.GetBestTarget().GetPoseAmbiguity() > kMaxAmbiguity) {
                _rejectedAmbiguous++;
                continue;
            }

            // Let PhotonPoseEstimator do the heavy lifting
            auto estimatedPose = unit->estimator.Update(result);
            if (!estimatedPose.has_value()) {
                _rejectedNoTargets++;
                continue;
            }

            // Compute rough distance to best tag for std dev scaling
            auto bestTransform = result.GetBestTarget().GetBestCameraToTarget();
            double distance = units::math::sqrt(
                units::math::pow<2>(bestTransform.X()) +
                units::math::pow<2>(bestTransform.Y()) +
                units::math::pow<2>(bestTransform.Z())
            ).value();

            // Sanity gate: reject poses outside field boundaries (16.46m x 8.21m)
            auto pose2d = estimatedPose->estimatedPose.ToPose2d();
            if (pose2d.X() < 0_m || pose2d.X() > 16.46_m ||
                pose2d.Y() < 0_m || pose2d.Y() > 8.21_m) {
                _rejectedOutOfBounds++;
                continue;
            }

            measurements.push_back(VisionMeasurement{
                pose2d,
                estimatedPose->timestamp,
                computeStdDevs(distance),
                std::string(unit->camera.GetCameraName())
            });

            // Telemetry: log each accepted pose so we can spot wrong-alliance measurements
            auto camName = std::string(unit->camera.GetCameraName());
            frc::SmartDashboard::PutNumber("Vision/" + camName + "/X (m)",  pose2d.X().value());
            frc::SmartDashboard::PutNumber("Vision/" + camName + "/Y (m)",  pose2d.Y().value());
            frc::SmartDashboard::PutNumber("Vision/" + camName + "/Rot (deg)", pose2d.Rotation().Degrees().value());

            _acceptedCount++;
        }
    }

    frc::SmartDashboard::PutNumber("Vision/Accepted", _acceptedCount);
    frc::SmartDashboard::PutNumber("Vision/Rejected OutOfBounds", _rejectedOutOfBounds);

    return measurements;
}

wpi::array<double, 3> VisionMulti::computeStdDevs(double distanceMeters) const {
    double xy    = 0.01 + (distanceMeters * 0.05);   // 1 cm base + 5 cm/m
    double theta = 0.01 + (distanceMeters * 0.02);   // 0.01 rad base + 0.02/m
    return {xy, xy, theta};
}

std::string VisionMulti::getStatus() {
    int activeCameras = 0;
    for (auto& unit : _cameras) {
        if (unit->camera.IsConnected()) {
            activeCameras++;
        }
    }
    return "VisionMulti - " + std::to_string(activeCameras) + "/4 cameras connected";
}

std::string VisionMulti::getLastTargets() {
    std::string info;
    for (auto& unit : _cameras) {
        auto results = unit->camera.GetAllUnreadResults();
        if (results.empty() || !results.back().HasTargets()) {
            continue;
        }
        info += "[" + std::string(unit->camera.GetCameraName()) + "] ";
        for (const auto& target : results.back().GetTargets()) {
            info += "ID=" + std::to_string(target.GetFiducialId()) + " ";
        }
    }
    return info.empty() ? "No targets" : info;
}

std::string VisionMulti::getRejectedCounts() {
    return "Accepted: " + std::to_string(_acceptedCount)
         + " | Rejected: NoTargets=" + std::to_string(_rejectedNoTargets)
         + " Stale=" + std::to_string(_rejectedStale)
         + " Ambiguous=" + std::to_string(_rejectedAmbiguous)
         + " OutOfBounds=" + std::to_string(_rejectedOutOfBounds);
}
