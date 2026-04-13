#include "field.hpp"
#include "visionmulti.hpp"
#include <frc/Timer.h>

namespace indy {

VisionMulti::VisionMulti()
    : _fieldLayout (field::layout())
{
    for (std::size_t i = 0; i < 1; ++i ) { // TODO: multicams: vision::kCameraNames.size(); ++i) {
        _cameras[i] = std::make_unique<CameraUnit> (
            vision::kCameraNames[i],
            vision::kRobotToCamera[i],
            _fieldLayout);
    }
}

void VisionMulti::readMeasurements (const frc::Pose2d& currentPose)
{
    for (std::size_t i = 0; i < _cameras.size(); ++i) {
        _rawResults[i] = _cameras[i]->camera.GetAllUnreadResults();
        processResults (std::string (_cameras[i]->camera.GetCameraName()),
                        _cameras[i]->estimator,
                        _rawResults[i],
                        currentPose);
    }
}

std::string VisionMulti::getStatus()
{
    int active = 0;
    for (auto& unit : _cameras) {
        if (unit->camera.IsConnected())
            active++;
    }
    return "VisionMulti - " + std::to_string (active) + "/4 cameras connected";
}

std::string VisionMulti::getLastTargets()
{
    std::string info;
    for (std::size_t i = 0; i < _cameras.size(); ++i) {
        const auto& results = _rawResults[i];
        if (results.empty() || ! results.back().HasTargets())
            continue;
        info += "[" + std::string (_cameras[i]->camera.GetCameraName()) + "] ";
        for (const auto& target : results.back().GetTargets()) {
            info += "ID=" + std::to_string (target.GetFiducialId()) + " ";
        }
    }
    return info.empty() ? "No targets" : info;
}

} // namespace indy
