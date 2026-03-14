#include "vision.hpp"
#include "frc/smartdashboard/SmartDashboard.h"

namespace indy {

VisionIO::VisionIO() {
    _measurements.reserve(vision::kCameraNames.size() * 16);
    _candidates.reserve(16);
}

std::string VisionIO::getRejectedCounts() {
    return "Accepted: " + std::to_string(_acceptedCount)
         + " | Rejected: NoTargets=" + std::to_string(_rejectedNoTargets)
         + " Stale=" + std::to_string(_rejectedStale)
         + " Ambiguous=" + std::to_string(_rejectedAmbiguous)
         + " OutOfBounds=" + std::to_string(_rejectedOutOfBounds)
         + " Velocity=" + std::to_string(_rejectedVelocity);
}

void VisionIO::processResults(const std::string& cameraName,
                        photon::PhotonPoseEstimator& estimator,
                        std::vector<photon::PhotonPipelineResult>& results,
                        const frc::Pose2d& currentPose)
    {
        _candidates.clear();

        for (auto& result : results) {
            if (!result.HasTargets()) {
                _rejectedNoTargets++;
                continue;
            }

            if (result.GetLatency() > kMaxLatency) {
                _rejectedStale++;
                continue;
            }

            // Only accept multi-tag PNP solves (≥2 tags visible).
            // Single-tag solves have an inherent 180° pose ambiguity and produce
            // drastic jumps — reject them outright regardless of ambiguity score.
            int tagCount = static_cast<int>(result.GetTargets().size());
            if (tagCount < kMinTagsForSingleSolve) {
                _rejectedAmbiguous++;
                continue;
            }

            auto estimatedPose = estimator.Update(result);
            if (!estimatedPose.has_value()) {
                _rejectedNoTargets++;
                continue;
            }

            auto bestTransform = result.GetBestTarget().GetBestCameraToTarget();
            double distance = units::math::sqrt(
                units::math::pow<2>(bestTransform.X()) +
                units::math::pow<2>(bestTransform.Y()) +
                units::math::pow<2>(bestTransform.Z())
            ).value();

            if (distance > kMaxTagDistance) {
                _rejectedOutOfBounds++;
                continue;
            }

            auto pose2d = estimatedPose->estimatedPose.ToPose2d();

            // Field bounds: 2026 Rebuilt AndyMark = 16.518 × 8.043 m (+0.5 m margin)
            if (pose2d.X() < -0.5_m || pose2d.X() > 17.018_m ||
                pose2d.Y() < -0.5_m || pose2d.Y() > 8.543_m) {
                _rejectedOutOfBounds++;
                continue;
            }

            // Odometry residual gate — the most important match-safety check.
            // If vision disagrees with current odometry by more than kMaxResidual,
            // the solve is probably bad. Don't let it corrupt the estimator.
            units::meter_t residual = pose2d.Translation().Distance(
                currentPose.Translation());
            const auto residualLimit = _dropouts > 10 ? kLooseResidual : kMaxResidual;
            if (residual > residualLimit) {
                _rejectedResidual++;
                frc::SmartDashboard::PutNumber(
                    "Vision/" + cameraName + "/Residual (m)", residual.value());
                continue;
            }

            // Velocity gate — reject solves that imply physically impossible robot motion.
            // Compares this pose against the last *committed* accepted pose for this camera.
            // State is updated only when a candidate actually wins (below), not here, so
            // a superseded candidate from the same cycle never poisons the next cycle's check.
            auto& camState = _cameraState[cameraName];
            if (camState.lastTime > 0_s) {
                auto dt = estimatedPose->timestamp - camState.lastTime;
                if (dt > 0_s) {
                    auto displacement = pose2d.Translation().Distance(camState.lastPose.Translation());
                    auto impliedVelocity = displacement / dt;
                    if (impliedVelocity > kMaxImpliedVelocity) {
                        _rejectedVelocity++;
                        continue;
                    }
                }
            }

            _candidates.push_back(Candidate{
                VisionMeasurement{
                    pose2d,
                    estimatedPose->timestamp,
                    computeStdDevs(distance, tagCount),
                    cameraName
                },
                tagCount,
                distance
            });
        }

        // Pick the single best candidate: prefer more tags, then closer distance.
        // Fusing more than one measurement per cycle per camera adds noise.
        if (!_candidates.empty()) {
            auto best = std::max_element(_candidates.begin(), _candidates.end(),
                [](const Candidate& a, const Candidate& b) {
                    if (a.tagCount != b.tagCount)
                        return a.tagCount < b.tagCount;
                    return a.distance > b.distance; // smaller distance = better
                });

            _measurements.push_back(best->measurement);
            _acceptedCount++;

            // Commit this pose as the velocity gate baseline for the next cycle.
            _cameraState[cameraName].lastPose = best->measurement.pose;
            _cameraState[cameraName].lastTime = best->measurement.timestamp;

#if BOT_TRACE_VISION
            const auto& pose2d = best->measurement.pose;
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/X (m)",        pose2d.X().value());
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Y (m)",        pose2d.Y().value());
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Rot (deg)",    pose2d.Rotation().Degrees().value());
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Distance (m)", best->distance);
            frc::SmartDashboard::PutNumber("Vision/" + cameraName + "/Tags",         best->tagCount);
#endif
        }

        // update num dropouts
        if (_measurements.empty()) {
            ++_dropouts;
            frc::SmartDashboard::PutNumber("Vision/Dropouts", (int)_dropouts);
        } else {
            _dropouts = 0;
        }
#if 1 //BOT_TRACE_VISION
        frc::SmartDashboard::PutNumber("Vision/Accepted",          _acceptedCount);
        frc::SmartDashboard::PutNumber("Vision/Rejected Residual", _rejectedResidual);
        frc::SmartDashboard::PutNumber("Vision/Rejected OutOfBounds", _rejectedOutOfBounds);
#endif
    }
}