#pragma once

#include "vision.hpp"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <array>
#include <memory>
#include <vector>

namespace indy {

/** Simulation-only vision implementation using PhotonVision's VisionSystemSim.
 
    Mirrors the four chassis-mounted cameras from `VisionMulti`, but drives them
    through `photon::VisionSystemSim` / `photon::PhotonCameraSim` instead of real
    hardware. All measurement processing is handled by `VisionIO::processResults()`.

    Call `update(robotPose)` **once per SimulationPeriodic** to advance the sim,
    then `getMeasurements()` flows through the normal `VisionIO` path unchanged.
*/
class VisionSim : public vision::VisionIO {
public:
    VisionSim();

    /** Advance the simulation with the current ground-truth robot pose.
     
        Must be called from `SimulationPeriodic` before `getMeasurements()`.
        
        @param robotPose Ground-truth 2D robot pose from the drivetrain simulation.
    */
    void update(const frc::Pose2d& robotPose);

    std::string getStatus() override;
    std::string getLastTargets() override;

protected:
    void readMeasurements(const frc::Pose2d& currentPose) override;

private:
    struct CameraUnit {
        photon::PhotonCamera camera;
        photon::PhotonPoseEstimator estimator;
        photon::SimCameraProperties props;
        std::shared_ptr<photon::PhotonCameraSim> cameraSim;

        CameraUnit(std::string_view name,
                   const frc::Transform3d& robotToCamera,
                   const frc::AprilTagFieldLayout& layout,
                   photon::VisionSystemSim& visionSim);
    };

    frc::AprilTagFieldLayout _fieldLayout;
    photon::VisionSystemSim _visionSim;
    std::array<std::unique_ptr<CameraUnit>, 2> _cameras;
};

}
