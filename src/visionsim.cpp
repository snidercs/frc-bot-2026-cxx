#include "visionsim.hpp"

// ─── CameraUnit ─────────────────────────────────────────────────────────────

VisionSim::CameraUnit::CameraUnit(std::string_view name,
                                   const frc::Transform3d& robotToCamera,
                                   const frc::AprilTagFieldLayout& layout,
                                   photon::VisionSystemSim& visionSim)
    : camera(std::string(name))
    , estimator(layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera)
{
    // ThriftyCam approximation: 70° diagonal FOV, 1280×720, ~50 ms latency
    props.SetCalibration(1280, 720, frc::Rotation2d{70_deg});
    props.SetCalibError(0.25, 0.08);
    props.SetFPS(30_Hz);
    props.SetAvgLatency(50_ms);
    props.SetLatencyStdDev(15_ms);

    cameraSim = std::make_shared<photon::PhotonCameraSim>(&camera, props);
    cameraSim->EnableDrawWireframe(true);

    visionSim.AddCamera(cameraSim.get(), robotToCamera);

    // Fall back to single-tag solve when only one tag is visible
    estimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
}

// ─── VisionSim ──────────────────────────────────────────────────────────────

VisionSim::VisionSim()
    : _fieldLayout(vision::getFieldLayout())
    , _visionSim("VisionSim")
{
    // No real coprocessor in sim — disable NT version checks to suppress warnings
    photon::PhotonCamera::SetVersionCheckEnabled(false);

    _visionSim.AddAprilTags(_fieldLayout);

    for (std::size_t i = 0; i < vision::kCameraNames.size(); ++i) {
        _cameras[i] = std::make_unique<CameraUnit>(
            vision::kCameraNames[i],
            vision::kRobotToCamera[i],
            _fieldLayout,
            _visionSim);
    }

    _measurements.reserve(vision::kCameraNames.size() * 16);
}

void VisionSim::update(const frc::Pose2d& robotPose) {
    _visionSim.Update(frc::Pose3d{robotPose});
}

const std::vector<VisionMeasurement>& VisionSim::getMeasurements() {
    _measurements.clear();

    for (auto& unit : _cameras) {
        auto results = unit->camera.GetAllUnreadResults();
        processResults(std::string(unit->camera.GetCameraName()),
                       unit->estimator,
                       results);
    }

    return _measurements;
}

std::string VisionSim::getStatus() {
    return "VisionSim - " + std::to_string(_cameras.size()) + "/2 cameras simulated";
}

std::string VisionSim::getLastTargets() {
    return "See SmartDashboard Vision/* entries";
}
