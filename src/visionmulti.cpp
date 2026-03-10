#include "visionmulti.hpp"
#include "photon/PhotonPoseEstimator.h"
#include <frc/Timer.h>

namespace indy {

VisionMulti::VisionMulti()
    : _fieldLayout(vision::getFieldLayout())
{
    for (std::size_t i = 0; i < vision::kCameraNames.size(); ++i) {
        _cameras[i] = std::make_unique<CameraUnit>(
            vision::kCameraNames[i],
            vision::kRobotToCamera[i],
            _fieldLayout);
    }
    _measurements.reserve(vision::kCameraNames.size() * 16);
}

const std::vector<VisionMeasurement>& VisionMulti::getMeasurements() {
    _measurements.clear();

    // NOTE: _fieldLayout always uses the default blue-origin (kBlueAllianceWallRightSide).
    // WPILib's pose estimator (and CTRE's AddVisionMeasurement) always expect poses in
    // the blue-origin frame regardless of alliance. Setting the origin to red would cause
    // PhotonPoseEstimator to output red-relative poses (~0–2 m) that the drivetrain
    // would interpret as being near the blue wall (~16 m), corrupting the pose estimate.
    for (std::size_t i = 0; i < _cameras.size(); ++i) {
        _rawResults[i] = _cameras[i]->camera.GetAllUnreadResults();
        processResults(std::string(_cameras[i]->camera.GetCameraName()),
                       _cameras[i]->estimator,
                       _rawResults[i]);
    }

    return _measurements;
}

std::string VisionMulti::getStatus() {
    int active = 0;
    for (auto& unit : _cameras) {
        if (unit->camera.IsConnected()) active++;
    }
    return "VisionMulti - " + std::to_string(active) + "/4 cameras connected";
}

std::string VisionMulti::getLastTargets() {
    std::string info;
    for (std::size_t i = 0; i < _cameras.size(); ++i) {
        const auto& results = _rawResults[i];
        if (results.empty() || !results.back().HasTargets()) continue;
        info += "[" + std::string(_cameras[i]->camera.GetCameraName()) + "] ";
        for (const auto& target : results.back().GetTargets()) {
            info += "ID=" + std::to_string(target.GetFiducialId()) + " ";
        }
    }
    return info.empty() ? "No targets" : info;
}

}
