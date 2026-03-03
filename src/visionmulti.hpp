#pragma once

#include "vision.hpp"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/DriverStation.h>
#include <array>
#include <memory>
#include <optional>

/** Multi-camera vision implementation using all four chassis-mounted cameras.
 
    Each camera has its own `PhotonPoseEstimator` seeded with the camera's
    robot-relative transform from `vision::kRobotToCamera`. All cameras are
    polled every cycle; results that pass gating (latency, ambiguity) are
    returned as individual `VisionMeasurement` entries for the pose estimator
    to fuse via `AddVisionMeasurement()`.
*/
class VisionMulti : public VisionIO {
public:
    VisionMulti();

    /** Returns all valid pose measurements from all cameras this cycle.
     
        @return Vector of measurements that passed gating; may be empty.
    */
    std::vector<VisionMeasurement> getMeasurements() override;

    std::string getStatus() override;
    std::string getLastTargets() override;
    std::string getRejectedCounts() override;

private:
    static constexpr double kMaxAmbiguity = 0.3;
    static constexpr units::second_t kMaxLatency = 0.5_s;

    // One camera + estimator pair per physical camera
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

    // Track last-seen alliance so we only call SetOrigin() when it changes
    std::optional<frc::DriverStation::Alliance> _lastAlliance;

    // Rejection counters (across all cameras)
    int _rejectedNoTargets  = 0;
    int _rejectedStale      = 0;
    int _rejectedAmbiguous  = 0;
    int _rejectedOutOfBounds = 0;
    int _acceptedCount      = 0;

    /** Scales pose std devs with distance: closer = more trusted. */
    wpi::array<double, 3> computeStdDevs(double distanceMeters) const;
};
