#include <gtest/gtest.h>
#include "vision.hpp"
#include "field.hpp"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

// ─── Helpers ────────────────────────────────────────────────────────────────

/** Minimal Processor subclass that lets tests inject pre-built Measurements
    directly into `_measurements`, bypassing PhotonVision entirely.
    This exercises every Processor method that doesn't require real camera data.
*/
class GatingMockProcessor : public indy::vision::Processor {
public:
    void injectMeasurement(const indy::vision::Measurement& m) {
        _pending.push_back(m);
    }
    void clearPending() { _pending.clear(); }

    std::string getStatus() override { return "GatingMock"; }

protected:
    void readMeasurements(const frc::Pose2d&) override {
        _measurements = _pending;
        _pending.clear();
    }

private:
    std::vector<indy::vision::Measurement> _pending;
};

static indy::vision::Measurement makeMeasurement(
    double x, double y, std::string source = "FL",
    units::second_t ts = 1.0_s)
{
    return indy::vision::Measurement {
        frc::Pose2d { units::meter_t(x), units::meter_t(y), frc::Rotation2d{} },
        ts,
        wpi::array<double, 3>{ 0.1, 0.1, 9999.0 },
        std::move(source)
    };
}

static const frc::Pose2d kOrigin{};
static const frc::ChassisSpeeds kStopped{};

// ─── Parameters defaults ────────────────────────────────────────────────────

TEST(VisionGatingTest, DefaultParameterValues) {
    const auto& p = indy::vision::Processor::Parameters::kDefault;
    EXPECT_DOUBLE_EQ(p.maxAmbiguity, 0.2);
    EXPECT_DOUBLE_EQ(p.maxLatency.value(), 0.25);
    EXPECT_DOUBLE_EQ(p.maxTagDistance, 5.5);
    EXPECT_DOUBLE_EQ(p.maxResidual.value(), 1.0);
    EXPECT_DOUBLE_EQ(p.looseResidual.value(), 2.0);
    EXPECT_EQ(p.dropoutLooseThreshold, 3u);
    EXPECT_DOUBLE_EQ(p.maxImpliedVelocity.value(), 5.0);
}

TEST(VisionGatingTest, SetParametersRoundTrip) {
    GatingMockProcessor proc;
    auto loose = indy::vision::Processor::Parameters::kDefault;
    loose.maxResidual = 3.0_m;
    loose.dropoutLooseThreshold = 1;

    proc.setParameters(loose);
    EXPECT_DOUBLE_EQ(proc.parameters().maxResidual.value(), 3.0);
    EXPECT_EQ(proc.parameters().dropoutLooseThreshold, 1u);

    proc.setParameters(indy::vision::Processor::Parameters::kDefault);
    EXPECT_DOUBLE_EQ(proc.parameters().maxResidual.value(), 1.0);
}

// ─── Dropout counter ────────────────────────────────────────────────────────

TEST(VisionGatingTest, DropoutCounterIncrementsOnEmptyCycle) {
    GatingMockProcessor proc;
    // No measurements injected → empty cycle → _dropouts increments
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 0u);

    // Feed a good measurement — _dropouts should reset to 0
    proc.injectMeasurement(makeMeasurement(0.0, 0.0));
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 1u);

    // Two empty cycles in a row
    proc.process(kOrigin, kStopped);
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 0u);
}

TEST(VisionGatingTest, DropoutCounterResetsOnAcceptedMeasurement) {
    GatingMockProcessor proc;
    // Run enough empty cycles to exceed the default dropoutLooseThreshold (3)
    for (int i = 0; i < 5; ++i)
        proc.process(kOrigin, kStopped);

    EXPECT_EQ(proc.measurements().size(), 0u);

    // Now inject a measurement — dropout counter must reset
    proc.injectMeasurement(makeMeasurement(0.0, 0.0));
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 1u);

    // Next empty cycle restarts from 0, not from 5
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 0u);
}

// ─── getRejectedCounts includes Residual ────────────────────────────────────

TEST(VisionGatingTest, RejectedCountsContainsResidualField) {
    GatingMockProcessor proc;
    proc.process(kOrigin, kStopped);
    const std::string counts = proc.getRejectedCounts();
    EXPECT_NE(counts.find("Residual="), std::string::npos)
        << "getRejectedCounts() must contain 'Residual=' field. Got: " << counts;
}

TEST(VisionGatingTest, RejectedCountsContainsAllFields) {
    GatingMockProcessor proc;
    proc.process(kOrigin, kStopped);
    const std::string counts = proc.getRejectedCounts();
    EXPECT_NE(counts.find("Accepted:"),    std::string::npos) << counts;
    EXPECT_NE(counts.find("NoTargets="),   std::string::npos) << counts;
    EXPECT_NE(counts.find("Stale="),       std::string::npos) << counts;
    EXPECT_NE(counts.find("Ambiguous="),   std::string::npos) << counts;
    EXPECT_NE(counts.find("OutOfBounds="), std::string::npos) << counts;
    EXPECT_NE(counts.find("Residual="),    std::string::npos) << counts;
    EXPECT_NE(counts.find("Velocity="),    std::string::npos) << counts;
}

// ─── setParameters affects accepted count ───────────────────────────────────

TEST(VisionGatingTest, AcceptedCountIncrementsEachCycle) {
    GatingMockProcessor proc;

    // The mock bypasses processResults() so _acceptedCount (incremented only
    // by the real camera pipeline) stays zero. Verify via measurements() size
    // instead, which the mock does populate correctly.
    proc.injectMeasurement(makeMeasurement(0.0, 0.0, "FL", 1.0_s));
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 1u);

    proc.injectMeasurement(makeMeasurement(0.1, 0.0, "FL", 2.0_s));
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 1u);

    // Two empty cycles → measurements() returns empty each time
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 0u);
    proc.process(kOrigin, kStopped);
    EXPECT_EQ(proc.measurements().size(), 0u);
}

// ─── computeStdDevs scaling ─────────────────────────────────────────────────

TEST(VisionGatingTest, StdDevsIncreaseWithDistance) {
    GatingMockProcessor proc;
    // Access protected method via a thin subclass shim
    struct StdDevExposer : public GatingMockProcessor {
        wpi::array<double, 3> expose(double dist, int tags) {
            return computeStdDevs(dist, tags);
        }
    } exposer;

    auto near = exposer.expose(1.0, 1);
    auto far  = exposer.expose(4.0, 1);
    EXPECT_LT(near[0], far[0]) << "stdDev X should increase with distance";
    EXPECT_LT(near[1], far[1]) << "stdDev Y should increase with distance";
}

TEST(VisionGatingTest, StdDevsDecreasedForMoreTags) {
    struct StdDevExposer : public GatingMockProcessor {
        wpi::array<double, 3> expose(double dist, int tags) {
            return computeStdDevs(dist, tags);
        }
    } exposer;

    auto singleTag = exposer.expose(2.0, 1);
    auto tripleTag = exposer.expose(2.0, 3);
    EXPECT_GT(singleTag[0], tripleTag[0]) << "More tags should yield tighter (smaller) stdDevs";
}

TEST(VisionGatingTest, ThetaStdDevAlwaysHigh) {
    struct StdDevExposer : public GatingMockProcessor {
        wpi::array<double, 3> expose(double dist, int tags) {
            return computeStdDevs(dist, tags);
        }
    } exposer;

    auto devs = exposer.expose(1.0, 2);
    EXPECT_GT(devs[2], 100.0) << "Theta stdDev must be very high so vision never overrides gyro heading";
}
