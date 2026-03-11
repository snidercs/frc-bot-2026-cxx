#include <gtest/gtest.h>
#include "vision.hpp"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

/** Mock implementation of VisionIO for testing */
class MockVisionIO : public indy::VisionIO {
private:
    int _callCount = 0;

public:
    void addMeasurement(const indy::VisionMeasurement& measurement) {
        _measurements.push_back(measurement);
    }

    void clearMeasurements() {
        _measurements.clear();
    }

    const std::vector<indy::VisionMeasurement>& getMeasurements(
            const frc::Pose2d& /*currentPose*/) override {
        _callCount++;
        return _measurements;
    }

    std::string getStatus() override {
        return "Mock Vision - " + std::to_string(_callCount) + " calls";
    }

    int getCallCount() const { return _callCount; }
};

// Test VisionMeasurement struct creation
TEST(VisionTest, MeasurementCreation) {
    indy::VisionMeasurement measurement{
        frc::Pose2d{1.0_m, 2.0_m, frc::Rotation2d{45_deg}},
        1.5_s,
        wpi::array<double, 3>{0.1, 0.1, 0.05},
        "FL"
    };

    EXPECT_DOUBLE_EQ(measurement.pose.X().value(), 1.0);
    EXPECT_DOUBLE_EQ(measurement.pose.Y().value(), 2.0);
    EXPECT_DOUBLE_EQ(measurement.pose.Rotation().Degrees().value(), 45.0);
    EXPECT_DOUBLE_EQ(measurement.timestamp.value(), 1.5);
    EXPECT_DOUBLE_EQ(measurement.stdDevs[0], 0.1);
    EXPECT_DOUBLE_EQ(measurement.stdDevs[1], 0.1);
    EXPECT_DOUBLE_EQ(measurement.stdDevs[2], 0.05);
    EXPECT_EQ(measurement.source, "FL");
}

// Test VisionIO interface with mock
TEST(VisionTest, MockVisionIO) {
    MockVisionIO mockVision;
    const frc::Pose2d anyPose{};  // pose doesn't matter for mock — gating is bypassed

    // Initially empty
    const auto& measurements = mockVision.getMeasurements(anyPose);
    EXPECT_EQ(measurements.size(), 0);
    EXPECT_EQ(mockVision.getCallCount(), 1);

    // Add a measurement
    indy::VisionMeasurement m1{
        frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0_deg}},
        1.0_s,
        wpi::array<double, 3>{0.5, 0.5, 0.1},
        "FR"
    };
    mockVision.addMeasurement(m1);

    const auto& measurements2 = mockVision.getMeasurements(anyPose);
    EXPECT_EQ(measurements2.size(), 1);
    EXPECT_EQ(measurements2[0].source, "FR");
    EXPECT_EQ(mockVision.getCallCount(), 2);

    // Add multiple measurements
    indy::VisionMeasurement m2{
        frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{90_deg}},
        2.0_s,
        wpi::array<double, 3>{0.3, 0.3, 0.08},
        "BL"
    };
    mockVision.addMeasurement(m2);

    const auto& measurements3 = mockVision.getMeasurements(anyPose);
    EXPECT_EQ(measurements3.size(), 2);
    EXPECT_EQ(measurements3[1].source, "BL");

    // Clear and verify
    mockVision.clearMeasurements();
    const auto& measurements4 = mockVision.getMeasurements(anyPose);
    EXPECT_EQ(measurements4.size(), 0);
}

// Test vision constants
TEST(VisionTest, CameraNames) {
    EXPECT_EQ(indy::vision::kCameraNames.size(), 2);
    EXPECT_STREQ(indy::vision::kCameraNames[0], "FL");
    EXPECT_STREQ(indy::vision::kCameraNames[1], "BL");
}

TEST(VisionTest, CameraTransforms) {
    EXPECT_EQ(indy::vision::kRobotToCamera.size(), 2);

    // FL camera: forward, left, up
    const auto& flTransform = indy::vision::kRobotToCamera[0];
    EXPECT_GT(flTransform.X().value(), 0.0);  // Forward
    EXPECT_GT(flTransform.Y().value(), 0.0);  // Left
    EXPECT_GT(flTransform.Z().value(), 0.0);  // Up

    // BL camera: backward, left, up
    const auto& blTransform = indy::vision::kRobotToCamera[1];
    EXPECT_LT(blTransform.X().value(), 0.0);  // Backward
    EXPECT_GT(blTransform.Y().value(), 0.0);  // Left
    EXPECT_GT(blTransform.Z().value(), 0.0);  // Up
}

TEST(VisionTest, TurretPivot) {
    // Currently at origin (placeholder)
    EXPECT_DOUBLE_EQ(indy::vision::kTurretPivotInRobot.X().value(), 0.0);
    EXPECT_DOUBLE_EQ(indy::vision::kTurretPivotInRobot.Y().value(), 0.0);
}

TEST(VisionTest, FieldLayout) {
    auto layout = indy::vision::fieldLayout();
    // Field layout should have tags
    auto tags = layout.GetTags();
    EXPECT_GT(tags.size(), 0);
}

// Test debug methods
TEST(VisionTest, DebugMethods) {
    MockVisionIO mockVision;
    
    EXPECT_NE(mockVision.getStatus(), "");
    EXPECT_NE(mockVision.getLastTargets(), "");
    EXPECT_NE(mockVision.getRejectedCounts(), "");
}
