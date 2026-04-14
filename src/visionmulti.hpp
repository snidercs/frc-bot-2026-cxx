#pragma once

#include "vision.hpp"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <array>
#include <memory>
#include <vector>

namespace indy {

/** Multi-camera vision implementation using all four chassis-mounted cameras.
 
    Each camera has its own `PhotonPoseEstimator` seeded with the camera's
    robot-relative transform from `vision::kRobotToCamera`. All cameras are
    polled every cycle; results that pass gating (latency, ambiguity) are
    returned as individual `VisionMeasurement` entries for the pose estimator
    to fuse via `AddVisionMeasurement()`.

    All measurement processing is handled by `Processor::processResults()`.
    This class only owns the camera/estimator pairs and fetches raw results.
*/
class VisionMulti : public vision::Processor {
public:
    VisionMulti();

    std::string getStatus() override;
    std::string getLastTargets() override;

protected: 
    void readMeasurements(const frc::Pose2d& currentPose) override;

private:
    struct CameraUnit {
        photon::PhotonCamera camera;
        photon::PhotonPoseEstimator estimator;

        CameraUnit(std::string_view name, const frc::Transform3d& robotToCamera,
                   const frc::AprilTagFieldLayout& layout)
            : camera(std::string(name))
            , estimator(layout,
                        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamera)
        {
            // Fall back to single-tag solve when only one tag is visible.
            // CLOSEST_TO_REFERENCE_POSE uses the current odometry pose (set via
            // SetReferencePose() each cycle in processResults()) to pick the
            // geometrically closer of the two PNP mirror solutions, eliminating
            // the 180° flip ambiguity that LOWEST_AMBIGUITY cannot resolve.
            estimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::CLOSEST_TO_REFERENCE_POSE);
        }
    };

    frc::AprilTagFieldLayout _fieldLayout;
    std::array<std::unique_ptr<CameraUnit>, vision::kCameraNames.size()> _cameras;

    std::array<std::vector<photon::PhotonPipelineResult>, vision::kCameraNames.size()> _rawResults;
};

}
