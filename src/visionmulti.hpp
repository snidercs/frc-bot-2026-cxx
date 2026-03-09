#pragma once

#include "vision.hpp"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <array>
#include <memory>
#include <vector>

/** Multi-camera vision implementation using all four chassis-mounted cameras.
 
    Each camera has its own `PhotonPoseEstimator` seeded with the camera's
    robot-relative transform from `vision::kRobotToCamera`. All cameras are
    polled every cycle; results that pass gating (latency, ambiguity) are
    returned as individual `VisionMeasurement` entries for the pose estimator
    to fuse via `AddVisionMeasurement()`.

    All measurement processing is handled by `VisionIO::processResults()`.
    This class only owns the camera/estimator pairs and fetches raw results.
*/
class VisionMulti : public VisionIO {
public:
    VisionMulti();

    const std::vector<VisionMeasurement>& getMeasurements() override;
    std::string getStatus() override;
    std::string getLastTargets() override;

private:
    struct CameraUnit {
        photon::PhotonCamera camera;
        photon::PhotonPoseEstimator estimator;

        CameraUnit(std::string_view name, const frc::Transform3d& robotToCamera,
                   const frc::AprilTagFieldLayout& layout)
            : camera(std::string(name))
            , estimator(layout,
                        photon::PoseStrategy::LOWEST_AMBIGUITY,
                        robotToCamera)
        {}
    };

    frc::AprilTagFieldLayout _fieldLayout;
    std::array<std::unique_ptr<CameraUnit>, 4> _cameras;

    // Raw results cached each cycle so getLastTargets() can read without
    // re-consuming them. Always call getMeasurements() first.
    std::array<std::vector<photon::PhotonPipelineResult>, 4> _rawResults;
};
