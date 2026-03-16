#include <gtest/gtest.h>
#include "vision.hpp"
#include "field.hpp"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

/** Mock implementation of VisionIO for testing.
 
    Implements the `readMeasurements()` hook so tests can pre-load measurements
    via `addMeasurement()` and then drive the pipeline through `read()` /
    `measurements()` exactly as production code does.
*/
class MockVisionIO : public indy::vision::VisionIO {
private:
    int _callCount = 0;
    std::vector<indy::vision::Measurement> _pending;

public:
    /** Stage a measurement to be returned on the next `process()` call. */
    void addMeasurement(const indy::vision::Measurement& measurement) {
        _pending.push_back(measurement);
    }

    /** Clear all staged and committed measurements. */
    void clearMeasurements() {
        _pending.clear();
        _measurements.clear();
    }

    std::string getStatus() override {
        return "Mock Vision - " + std::to_string(_callCount) + " calls";
    }

    int getCallCount() const { return _callCount; }

protected:
    /** Satisfies the pure-virtual hook. Moves staged measurements into the
        shared `_measurements` buffer that `read()` exposes via `measurements()`. */
    void readMeasurements(const frc::Pose2d& /*currentPose*/) override {
        _callCount++;
        _measurements = _pending;
        _pending.clear();
    }
};

// Test Measurement struct creation
TEST(VisionTest, MeasurementCreation) {
    indy::vision::Measurement measurement{
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
    const frc::Pose2d anyPose{};
    const frc::ChassisSpeeds anySpeeds{};

    // Initially empty
    mockVision.process(anyPose, anySpeeds);
    EXPECT_EQ(mockVision.measurements().size(), 0);
    EXPECT_EQ(mockVision.getCallCount(), 1);

    // Add a measurement and read it
    indy::vision::Measurement m1{
        frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0_deg}},
        1.0_s,
        wpi::array<double, 3>{0.5, 0.5, 0.1},
        "FR"
    };
    mockVision.addMeasurement(m1);

    mockVision.process(anyPose, anySpeeds);
    EXPECT_EQ(mockVision.measurements().size(), 1);
    EXPECT_EQ(mockVision.measurements()[0].source, "FR");
    EXPECT_EQ(mockVision.getCallCount(), 2);

    // Stage two measurements and read them
    indy::vision::Measurement m2{
        frc::Pose2d{1.0_m, 1.0_m, frc::Rotation2d{90_deg}},
        2.0_s,
        wpi::array<double, 3>{0.3, 0.3, 0.08},
        "BL"
    };
    mockVision.addMeasurement(m1);
    mockVision.addMeasurement(m2);

    mockVision.process(anyPose, anySpeeds);
    EXPECT_EQ(mockVision.measurements().size(), 2);
    EXPECT_EQ(mockVision.measurements()[1].source, "BL");

    // Clear and verify
    mockVision.clearMeasurements();
    mockVision.process(anyPose, anySpeeds);
    EXPECT_EQ(mockVision.measurements().size(), 0);
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
    // -7.5 in, 0.75 in from robot centre (measured turret pivot location)
    EXPECT_NEAR(indy::vision::kTurretPivotInRobot.X().value(), -7.5 * 0.0254, 1e-6);
    EXPECT_NEAR(indy::vision::kTurretPivotInRobot.Y().value(),  0.75 * 0.0254, 1e-6);
}

TEST(VisionTest, FieldLayout) {
    auto layout = indy::field::layout();
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
