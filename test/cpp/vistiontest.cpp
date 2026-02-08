#include <gtest/gtest.h>
#include "vision.hpp"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

/** Mock implementation of VisionIO for testing */
class MockVisionIO : public VisionIO {
private:
    std::vector<VisionMeasurement> _measurements;
    int _callCount = 0;

public:
    void addMeasurement(const VisionMeasurement& measurement) {
        _measurements.push_back(measurement);
    }

    void clearMeasurements() {
        _measurements.clear();
    }

    std::vector<VisionMeasurement> getMeasurements() override {
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
    VisionMeasurement measurement{
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

    // Initially empty
    auto measurements = mockVision.getMeasurements();
    EXPECT_EQ(measurements.size(), 0);
    EXPECT_EQ(mockVision.getCallCount(), 1);

    // Add a measurement
    VisionMeasurement m1{
        frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0_deg}},
        1.0_s,
        wpi::array<double, 3>{0.5, 0.5, 0.1},
        "FR"
    };
    mockVision.addMeasurement(m1);

    measurements = mockVision.getMeasurements();
    EXPECT_EQ(measurements.size(), 1);
    EXPECT_EQ(measurements[0].source, "FR");
    EXPECT_EQ(mockVision.getCallCount(), 2);

    // Add multiple measurements
    VisionMeasurement m2{
        frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{90_deg}},
        2.0_s,
        wpi::array<double, 3>{0.3, 0.3, 0.08},
        "BL"
    };
    mockVision.addMeasurement(m2);

    measurements = mockVision.getMeasurements();
    EXPECT_EQ(measurements.size(), 2);
    EXPECT_EQ(measurements[1].source, "BL");

    // Clear and verify
    mockVision.clearMeasurements();
    measurements = mockVision.getMeasurements();
    EXPECT_EQ(measurements.size(), 0);
}

// Test vision constants
TEST(VisionTest, CameraNames) {
    EXPECT_EQ(vision::kCameraNames.size(), 4);
    EXPECT_STREQ(vision::kCameraNames[0], "FL");
    EXPECT_STREQ(vision::kCameraNames[1], "FR");
    EXPECT_STREQ(vision::kCameraNames[2], "BL");
    EXPECT_STREQ(vision::kCameraNames[3], "BR");
}

TEST(VisionTest, CameraTransforms) {
    EXPECT_EQ(vision::kRobotToCamera.size(), 4);
    
    // Check FL camera is front-left
    const auto& flTransform = vision::kRobotToCamera[0];
    EXPECT_GT(flTransform.X().value(), 0.0);  // Forward
    EXPECT_GT(flTransform.Y().value(), 0.0);  // Left
    EXPECT_GT(flTransform.Z().value(), 0.0);  // Up
    
    // Check FR camera is front-right
    const auto& frTransform = vision::kRobotToCamera[1];
    EXPECT_GT(frTransform.X().value(), 0.0);  // Forward
    EXPECT_LT(frTransform.Y().value(), 0.0);  // Right
    EXPECT_GT(frTransform.Z().value(), 0.0);  // Up
}

TEST(VisionTest, TurretPivot) {
    // Currently at origin (placeholder)
    EXPECT_DOUBLE_EQ(vision::kTurretPivotInRobot.X().value(), 0.0);
    EXPECT_DOUBLE_EQ(vision::kTurretPivotInRobot.Y().value(), 0.0);
}

TEST(VisionTest, FieldLayout) {
    auto layout = vision::getFieldLayout();
    // Field layout should have tags
    auto tags = layout.GetTags();
    EXPECT_GT(tags.size(), 0);
}

TEST(VisionTest, GoalTagIds) {
    EXPECT_GT(vision::kGoalTagIds.size(), 0);
    // Verify all IDs are positive
    for (int id : vision::kGoalTagIds) {
        EXPECT_GT(id, 0);
    }
}

// Test debug methods
TEST(VisionTest, DebugMethods) {
    MockVisionIO mockVision;
    
    EXPECT_NE(mockVision.getStatus(), "");
    EXPECT_NE(mockVision.getLastTargets(), "");
    EXPECT_NE(mockVision.getRejectedCounts(), "");
}
