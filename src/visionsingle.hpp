#pragma once

#include "vision.hpp"
#include <photon/PhotonCamera.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <memory>

/** Single-camera vision implementation for testing.
 
    Simplified VisionIO implementation for testing with one camera.
    Uses PhotonVision to detect AprilTags and provide basic pose estimates.
    Suitable for quick hardware validation before full 4-camera setup.
*/
class VisionIOSingle : public VisionIO {
public:
    /** Constructs a single-camera vision system.
     
        @param cameraName Name of the PhotonVision camera (must match PV config)
    */
    explicit VisionIOSingle(std::string_view cameraName);

    std::vector<VisionMeasurement> getMeasurements() override;
    std::string getStatus() override;
    std::string getLastTargets() override;
    std::string getRejectedCounts() override;
    
    /** Gets the yaw angle to the best target for simple tracking.
     
        @return Yaw angle in degrees (positive = right, negative = left), or 0 if no target
    */
    double getBestTargetYaw();

    /** Fetches new results from the camera and caches them.
     
        Call this once per cycle before using getStatus(), getLastTargets(),
        or getBestTargetYaw(). Prevents multiple GetAllUnreadResults() calls
        which would consume the queue and return empty on subsequent reads.
    */
    void updateResults();

private:
    // PhotonVision camera
    photon::PhotonCamera _camera;
    
    // Camera transform - center front, 4.5 inches off ground
    static constexpr frc::Transform3d kCameraTransform{
        frc::Translation3d{0.0_m, 0.0_m, 0.1143_m},  // 4.5 inches = 0.1143 meters
        frc::Rotation3d{0_deg, 0_deg, 0_deg}          // Facing straight forward
    };
    
    // Field layout for AprilTag positions
    frc::AprilTagFieldLayout _fieldLayout;
    
    // Rejection tracking for debugging
    int _rejectedNoTargets = 0;
    int _rejectedStale = 0;
    int _rejectedAmbiguous = 0;
    int _acceptedCount = 0;
    
    // Thresholds for measurement gating
    static constexpr double kMaxAmbiguity = 0.3;  // Reject if ambiguity > 30%
    static constexpr units::second_t kMaxLatency = 0.5_s;  // Reject if older than 500ms
    
    // Cached camera results (populated by updateResults(), read by accessors)
    std::vector<photon::PhotonPipelineResult> _cachedResults;
    
    bool isValidMeasurement(const photon::PhotonPipelineResult& result);
    VisionMeasurement createMeasurement(const photon::PhotonPipelineResult& result);
    wpi::array<double, 3> computeStdDevs(double distance) const;
};
