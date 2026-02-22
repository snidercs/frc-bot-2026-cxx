#include "visionsingle.hpp"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

VisionIOSingle::VisionIOSingle(std::string_view cameraName)
    : _camera(std::string(cameraName))
    , _fieldLayout(vision::getFieldLayout())
{
    frc::SmartDashboard::PutString("VisionSingle/Camera", std::string(cameraName));
}

std::vector<VisionMeasurement> VisionIOSingle::getMeasurements() {
    std::vector<VisionMeasurement> measurements;
    
    // Get all unread results from PhotonVision (2026 API)
    auto results = _camera.GetAllUnreadResults();
    
    if (results.empty()) {
        _rejectedNoTargets++;
        return measurements;  // Empty vector
    }
    
    // Use the most recent result
    auto result = results.back();
    
    // Check if we have valid targets
    if (!result.HasTargets()) {
        _rejectedNoTargets++;
        return measurements;
    }
    
    // Validate the measurement
    if (!isValidMeasurement(result)) {
        return measurements;  // Already incremented rejection counters
    }
    
    // Create and return the measurement
    measurements.push_back(createMeasurement(result));
    _acceptedCount++;
    
    return measurements;
}

bool VisionIOSingle::isValidMeasurement(const photon::PhotonPipelineResult& result) {
    // Get the best target
    auto target = result.GetBestTarget();
    
    // Check latency - reject if too old
    auto latency = result.GetLatency();
    if (latency > kMaxLatency) {
        _rejectedStale++;
        return false;
    }
    
    // Check pose ambiguity - reject if too uncertain
    if (target.GetPoseAmbiguity() > kMaxAmbiguity) {
        _rejectedAmbiguous++;
        return false;
    }
    
    return true;
}

VisionMeasurement VisionIOSingle::createMeasurement(const photon::PhotonPipelineResult& result) {
    // Initialize with defaults
    VisionMeasurement measurement{
        frc::Pose2d{},
        0_s,
        wpi::array<double, 3>{999.0, 999.0, 999.0},
        "TestCam"
    };
    
    // Get the best target
    auto target = result.GetBestTarget();
    auto fiducialId = target.GetFiducialId();
    
    // Try to get the tag pose from field layout
    auto tagPoseOptional = _fieldLayout.GetTagPose(fiducialId);
    
    if (tagPoseOptional.has_value()) {
        // Get tag pose in field coordinates
        auto tagPose3d = tagPoseOptional.value();
        
        // Get target transform (camera to tag)
        auto bestTransform = target.GetBestCameraToTarget();
        
        // Compute robot pose: tag pose - (robot to camera + camera to tag)
        // Robot to camera is the inverse of camera to robot
        auto cameraToRobot = kCameraTransform.Inverse();
        
        // Camera to tag (from PhotonVision)
        frc::Transform3d cameraToTag{
            frc::Translation3d{
                bestTransform.X(),
                bestTransform.Y(),
                bestTransform.Z()
            },
            frc::Rotation3d{
                bestTransform.Rotation().X(),
                bestTransform.Rotation().Y(),
                bestTransform.Rotation().Z()
            }
        };
        
        // Compute robot pose on field
        // Robot pose = Tag pose - (camera to tag + camera to robot offset)
        auto cameraPose3d = tagPose3d.TransformBy(cameraToTag.Inverse());
        auto robotPose3d = cameraPose3d.TransformBy(cameraToRobot);
        
        // Convert to 2D pose (drop Z coordinate)
        measurement.pose = robotPose3d.ToPose2d();
        
        // Compute distance to target for standard deviation scaling
        double distance = units::math::sqrt(
            units::math::pow<2>(bestTransform.X()) +
            units::math::pow<2>(bestTransform.Y()) +
            units::math::pow<2>(bestTransform.Z())
        ).value();
        
        measurement.stdDevs = computeStdDevs(distance);
    } else {
        // Unknown tag - use zero pose (invalid, but won't crash)
        measurement.pose = frc::Pose2d{};
        measurement.stdDevs = wpi::array<double, 3>{999.0, 999.0, 999.0};  // High uncertainty
    }
    
    // Timestamp (current time - latency)
    auto currentTime = frc::Timer::GetFPGATimestamp();
    measurement.timestamp = currentTime - result.GetLatency();
    
    // Source camera
    measurement.source = "TestCam";
    
    return measurement;
}

wpi::array<double, 3> VisionIOSingle::computeStdDevs(double distance) const {
    // Simple distance-based scaling
    // Close targets are more accurate, far targets less so
    double xyStdDev = 0.01 + (distance * 0.05);  // 1cm + 5cm per meter
    double thetaStdDev = 0.01 + (distance * 0.02);  // 0.01 rad + 0.02 per meter
    
    return {xyStdDev, xyStdDev, thetaStdDev};
}

void VisionIOSingle::updateResults() {
    _cachedResults = _camera.GetAllUnreadResults();
}

std::string VisionIOSingle::getStatus() {
    std::string status = "VisionIOSingle - ";
    
    if (!_cachedResults.empty() && _cachedResults.back().HasTargets()) {
        status += "Tracking (";
        status += std::to_string(_cachedResults.back().GetTargets().size());
        status += " targets)";
    } else {
        status += "No targets";
    }
    
    return status;
}

std::string VisionIOSingle::getLastTargets() {
    if (_cachedResults.empty()) {
        return "No results available";
    }
    
    auto result = _cachedResults.back();
    
    if (!result.HasTargets()) {
        return "No targets detected";
    }
    
    std::string info = "Targets: ";
    for (const auto& target : result.GetTargets()) {
        info += "ID=" + std::to_string(target.GetFiducialId());
        info += " Yaw=" + std::to_string(target.GetYaw());
        info += " Dist=" + std::to_string(target.GetBestCameraToTarget().X().value()) + "m, ";
    }
    
    return info;
}

std::string VisionIOSingle::getRejectedCounts() {
    std::string counts = "Accepted: " + std::to_string(_acceptedCount);
    counts += " | Rejected: NoTargets=" + std::to_string(_rejectedNoTargets);
    counts += " Stale=" + std::to_string(_rejectedStale);
    counts += " Ambiguous=" + std::to_string(_rejectedAmbiguous);
    return counts;
}

double VisionIOSingle::getBestTargetYaw() {
    if (_cachedResults.empty()) {
        frc::SmartDashboard::PutString("VisionSingle/Debug", "No results");
        return 0.0;
    }
    
    auto result = _cachedResults.back();
    
    if (!result.HasTargets()) {
        frc::SmartDashboard::PutString("VisionSingle/Debug", "No targets in result");
        return 0.0;
    }
    
    auto target = result.GetBestTarget();
    double yaw = target.GetYaw();
    int targetId = target.GetFiducialId();
    
    // Debug output
    frc::SmartDashboard::PutString("VisionSingle/Debug", 
        "Target ID=" + std::to_string(targetId) + " Yaw=" + std::to_string(yaw));
    frc::SmartDashboard::PutNumber("VisionSingle/TargetID", targetId);
    frc::SmartDashboard::PutNumber("VisionSingle/TargetYaw", yaw);
    frc::SmartDashboard::PutNumber("VisionSingle/TargetArea", target.GetArea());
    
    return yaw;
}
