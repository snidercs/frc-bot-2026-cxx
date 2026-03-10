#include "vision.hpp"

std::string indy::VisionIO::getRejectedCounts() {
    return "Accepted: " + std::to_string(_acceptedCount)
         + " | Rejected: NoTargets=" + std::to_string(_rejectedNoTargets)
         + " Stale=" + std::to_string(_rejectedStale)
         + " Ambiguous=" + std::to_string(_rejectedAmbiguous)
         + " OutOfBounds=" + std::to_string(_rejectedOutOfBounds);
}
