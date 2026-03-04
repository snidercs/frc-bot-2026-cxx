# Turret Shooter Lookup Table System

## Overview
Fast, real-time lookup system for determining shooter speed and turret angle based on robot state. Must be deterministic, allocation-free, and suitable for high-frequency robot loop execution (10-15ms target).

## Requirements
- **Performance**: < 50μs lookup time per iteration (target < 10μs)
- **No allocations**: All memory pre-allocated at initialization
- **No string lookups**: Integer/float indexing only
- **Update every loop**: 67-100Hz (10-15ms) update rate
- **Inputs**: Distance to target, robot velocity (x/y), robot angular velocity
- **Outputs**: Shooter RPM, turret angle compensation

## Design: Multi-Dimensional Lookup Tables

### Table Structure
Use fixed-size multi-dimensional arrays with linear interpolation between grid points.

```cpp
// In turret.hpp
class Turret {
private:
    // Lookup table dimensions
    static constexpr size_t kDistanceSteps = 20;      // 0-20m in 1m increments
    static constexpr size_t kVelocitySteps = 10;      // 0-5 m/s in 0.5 m/s increments
    static constexpr size_t kRotationSteps = 8;       // -180 to +180 deg/s
    
    // Pre-allocated lookup tables (compile-time constant)
    static constexpr std::array<std::array<std::array<double, kRotationSteps>, kVelocitySteps>, kDistanceSteps> 
        kShooterSpeedTable = /* ... */;
    
    static constexpr std::array<std::array<std::array<double, kRotationSteps>, kVelocitySteps>, kDistanceSteps> 
        kAngleCompensationTable = /* ... */;
    
    // Grid ranges for interpolation
    static constexpr double kMaxDistance = 20.0;      // meters
    static constexpr double kMaxVelocity = 5.0;       // m/s
    static constexpr double kMaxRotationRate = 180.0; // deg/s
    
    // Fast lookup with trilinear interpolation
    double lookupShooterSpeed(double distance, double velocity, double rotationRate) const;
    double lookupAngleCompensation(double distance, double velocity, double rotationRate) const;
};
```

### Lookup Algorithm (Trilinear Interpolation)

```cpp
double Turret::lookupShooterSpeed(double distance, double velocity, double rotationRate) const {
    // 1. Clamp inputs to valid range
    distance = std::clamp(distance, 0.0, kMaxDistance);
    velocity = std::clamp(velocity, 0.0, kMaxVelocity);
    rotationRate = std::clamp(rotationRate, -kMaxRotationRate, kMaxRotationRate);
    
    // 2. Convert to grid coordinates [0, steps-1]
    double d_idx = (distance / kMaxDistance) * (kDistanceSteps - 1);
    double v_idx = (velocity / kMaxVelocity) * (kVelocitySteps - 1);
    double r_idx = ((rotationRate + kMaxRotationRate) / (2.0 * kMaxRotationRate)) * (kRotationSteps - 1);
    
    // 3. Find grid cell corners
    size_t d0 = static_cast<size_t>(d_idx);
    size_t d1 = std::min(d0 + 1, kDistanceSteps - 1);
    size_t v0 = static_cast<size_t>(v_idx);
    size_t v1 = std::min(v0 + 1, kVelocitySteps - 1);
    size_t r0 = static_cast<size_t>(r_idx);
    size_t r1 = std::min(r0 + 1, kRotationSteps - 1);
    
    // 4. Compute interpolation weights
    double d_weight = d_idx - d0;
    double v_weight = v_idx - v0;
    double r_weight = r_idx - r0;
    
    // 5. Trilinear interpolation (8 corner values)
    double c000 = kShooterSpeedTable[d0][v0][r0];
    double c001 = kShooterSpeedTable[d0][v0][r1];
    double c010 = kShooterSpeedTable[d0][v1][r0];
    double c011 = kShooterSpeedTable[d0][v1][r1];
    double c100 = kShooterSpeedTable[d1][v0][r0];
    double c101 = kShooterSpeedTable[d1][v0][r1];
    double c110 = kShooterSpeedTable[d1][v1][r0];
    double c111 = kShooterSpeedTable[d1][v1][r1];
    
    // Interpolate along distance axis
    double c00 = c000 * (1 - d_weight) + c100 * d_weight;
    double c01 = c001 * (1 - d_weight) + c101 * d_weight;
    double c10 = c010 * (1 - d_weight) + c110 * d_weight;
    double c11 = c011 * (1 - d_weight) + c111 * d_weight;
    
    // Interpolate along velocity axis
    double c0 = c00 * (1 - v_weight) + c10 * v_weight;
    double c1 = c01 * (1 - v_weight) + c11 * v_weight;
    
    // Interpolate along rotation axis
    return c0 * (1 - r_weight) + c1 * r_weight;
}
```

**Complexity**: O(1) with ~50 arithmetic operations
**Performance**: ~50-100ns on modern ARM Cortex-A9 (RoboRIO)

## Alternative: Simpler 2D Table (Distance + Velocity Only)

If rotation compensation is minimal, use 2D table:

```cpp
// Simpler bilinear interpolation
static constexpr std::array<std::array<double, kVelocitySteps>, kDistanceSteps> kShooterSpeedTable2D;

double lookupShooterSpeed2D(double distance, double velocity) const {
    // Clamp inputs
    distance = std::clamp(distance, 0.0, kMaxDistance);
    velocity = std::clamp(velocity, 0.0, kMaxVelocity);
    
    // Grid coordinates
    double d_idx = (distance / kMaxDistance) * (kDistanceSteps - 1);
    double v_idx = (velocity / kMaxVelocity) * (kVelocitySteps - 1);
    
    // Grid cell
    size_t d0 = static_cast<size_t>(d_idx);
    size_t d1 = std::min(d0 + 1, kDistanceSteps - 1);
    size_t v0 = static_cast<size_t>(v_idx);
    size_t v1 = std::min(v0 + 1, kVelocitySteps - 1);
    
    // Weights
    double d_weight = d_idx - d0;
    double v_weight = v_idx - v0;
    
    // Bilinear interpolation
    double c00 = kShooterSpeedTable2D[d0][v0];
    double c01 = kShooterSpeedTable2D[d0][v1];
    double c10 = kShooterSpeedTable2D[d1][v0];
    double c11 = kShooterSpeedTable2D[d1][v1];
    
    double c0 = c00 * (1 - d_weight) + c10 * d_weight;
    double c1 = c01 * (1 - d_weight) + c11 * d_weight;
    
    return c0 * (1 - v_weight) + c1 * v_weight;
}
```

**Complexity**: O(1) with ~20 arithmetic operations
**Performance**: ~20-50ns

## Table Population Strategy

### Option 1: Hardcoded Empirical Data
```cpp
// Measured on actual robot, compile-time constant
static constexpr std::array<std::array<double, 10>, 20> kShooterSpeedTable2D = {{
    // Distance 0m: [velocity 0, 0.5, 1.0, ..., 4.5 m/s]
    {50.0, 51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0},
    // Distance 1m:
    {52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0},
    // ... 18 more rows
}};
```

### Option 2: Runtime Initialization from Config
```cpp
class Turret {
private:
    // Mutable tables (initialized once)
    std::array<std::array<double, kVelocitySteps>, kDistanceSteps> _shooterSpeedTable;
    
    void initializeTables() {
        // Load from config file or NetworkTables
        // Only called once in constructor
        for (size_t d = 0; d < kDistanceSteps; ++d) {
            for (size_t v = 0; v < kVelocitySteps; ++v) {
                double distance = (d * kMaxDistance) / (kDistanceSteps - 1);
                double velocity = (v * kMaxVelocity) / (kVelocitySteps - 1);
                
                // Load from config or compute initial guess
                _shooterSpeedTable[d][v] = computeShooterSpeed(distance, velocity);
            }
        }
    }
};
```

### Option 3: Physics-Based Fallback + Tuning
```cpp
double computeShooterSpeed(double distance, double velocity) const {
    // Projectile motion calculation as baseline
    // Can be refined with empirical corrections
    constexpr double g = 9.81;           // gravity
    constexpr double target_height = 2.0; // speaker height
    constexpr double shooter_height = 0.5;
    
    double height_diff = target_height - shooter_height;
    double launch_angle = 45.0 * M_PI / 180.0; // Fixed launch angle
    
    // Solve for initial velocity (simplified)
    double v0_squared = (g * distance * distance) / 
                        (2 * std::cos(launch_angle) * std::cos(launch_angle) * 
                         (distance * std::tan(launch_angle) - height_diff));
    
    double v0 = std::sqrt(v0_squared);
    
    // Convert m/s to motor RPS (depends on wheel diameter, compression)
    constexpr double wheel_diameter = 0.1016; // 4 inches in meters
    constexpr double compression_factor = 0.9; // 90% effective velocity
    
    double wheel_speed_mps = v0 / compression_factor;
    double wheel_rps = wheel_speed_mps / (M_PI * wheel_diameter);
    
    // Add velocity compensation (moving toward/away from target)
    wheel_rps += velocity * 0.5; // Empirical coefficient
    
    return wheel_rps;
}
```

## Integration into Periodic Loop

```cpp
void Turret::Periodic() {
    // Get robot state (from odometry/vision)
    auto robotPose = /* from swerve drivetrain */;
    auto robotVelocity = /* from chassis speeds */;
    auto targetPose = /* speaker position from field layout */;
    
    // Compute inputs
    double distance = robotPose.Translation().Distance(targetPose.Translation()).value();
    double velocity_toward_target = /* project velocity onto target direction */;
    double rotation_rate = robotVelocity.omega.value(); // deg/s
    
    // Fast lookup (no allocations, ~100ns)
    double shooter_speed = lookupShooterSpeed(distance, velocity_toward_target, rotation_rate);
    double angle_compensation = lookupAngleCompensation(distance, velocity_toward_target, rotation_rate);
    
    // Apply to motors
    if (_autoAimEnabled) {
        setShooterVelocity(units::turns_per_second_t{shooter_speed});
        setTargetAngle(computeAimAngle(robotPose, targetPose) + angle_compensation);
    }
    
    // Telemetry
    frc::SmartDashboard::PutNumber("Turret/Lookup Distance", distance);
    frc::SmartDashboard::PutNumber("Turret/Lookup Speed", shooter_speed);
}
```

## Memory Footprint

### 3D Table (20×10×8):
- **Size**: 20 × 10 × 8 × 8 bytes (double) = **12.8 KB** per table
- **Two tables** (speed + angle): **25.6 KB total**
- **Acceptable**: RoboRIO has 256 MB RAM

### 2D Table (20×10):
- **Size**: 20 × 10 × 8 bytes = **1.6 KB** per table
- **Two tables**: **3.2 KB total**
- **Excellent**: Fits in CPU cache for ultra-fast access

## Tuning Workflow

1. **Initial Population**: Physics-based estimates or conservative defaults
2. **Manual Testing**: Shoot from various distances/velocities, record results
3. **Table Updates**: Update grid points based on measured error
4. **NetworkTables Tuning**: 
   ```cpp
   // Allow runtime table updates via NT for testing
   frc::SmartDashboard::PutNumber("Turret/Table[5][3]", _shooterSpeedTable[5][3]);
   // In Periodic():
   _shooterSpeedTable[5][3] = frc::SmartDashboard::GetNumber("Turret/Table[5][3]", default);
   ```
5. **Persist to Config**: Export tuned table to config file or hardcode

## Performance Validation

Add performance telemetry:
```cpp
void Turret::Periodic() {
    auto start = std::chrono::high_resolution_clock::now();
    
    double speed = lookupShooterSpeed(distance, velocity, rotation);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    
    frc::SmartDashboard::PutNumber("Turret/Lookup Time (ns)", duration_ns);
}
```

**Target**: < 500ns (0.5μs) for 10ms loop
**Acceptable**: < 1000ns (1μs) for 15ms loop

With a 10ms loop budget and ~50-100ns lookup time, you're using < 0.001% of your loop time - plenty of headroom for other subsystems!

## Recommendations

1. **Start with 2D table** (distance + velocity) - simpler, faster, easier to tune
2. **Add rotation axis** only if testing shows significant impact
3. **Use physics-based initialization** for reasonable starting values
4. **Implement NetworkTables tuning** for rapid iteration during practice
5. **Cache robot velocity** - compute once per loop, not per subsystem
6. **Profile in real environment** - simulator won't reflect real performance

## References
- [Wikipedia: Trilinear Interpolation](https://en.wikipedia.org/wiki/Trilinear_interpolation)
- [WPILib Field Layout](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html)
- FRC Team 254: "Lookup tables for shooter compensation" (2019)
