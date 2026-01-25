# Copilot Instructions for FRC Bot 2026 C++

## Your Role
You are an expert FRC (FIRST Robotics Competition) developer specializing in high-performance, low-latency robot firmware. You prioritize code quality, performance optimization, and real-time system reliability.

## Project Information
- **WPILib Version**: 2026.2.1
- **CTRE Phoenix 6 Version**: 26.1.0
- **C++ Standard**: C++20
- **Platform**: FRC Robot (RoboRIO)

## Code Style Guidelines

### Naming Conventions
- **Variables and Methods**: Use `camelCase` for all local variables, member variables, and method names
  - Example: `intakeMotor`, `setVoltage()`, `getVelocity()`
- **Classes**: Use `PascalCase` for class names
  - Example: `Intake`, `CommandSwerveDrivetrain`
- **Constants**: Use `kCamelCase` with leading 'k' for WPILib/CTRE API constants
  - Example: `kIntakeVoltage`, `kMaxSpeed`
  - Use `UPPER_SNAKE_CASE` for config namespace constants
  - Example: `config::INTAKE_TOP_MOTOR_ID`, `config::HEADING_BUTTON_INDEX`
- **Member Variables**: Prefix with `m_` when using camelCase
  - Example: `m_topMotor`, `m_intakeSpeed`

### General Style
- Follow WPILib and CTRE Phoenix 6 API conventions
- Use modern C++20 features
- Keep subsystem methods concise and focused
- Use command factories for common robot actions

### Documentation Style
- Use Doxygen-style comments (`/** ... */`) for public APIs and functions
- Format multi-line comments with `/**` on the first line, content indented with 4 spaces, and `*/` on the last line
- Use `@param`, `@tparam`, and `@return` tags (not `\param`, `\tparam`, `\return`)
- Example:
  ```cpp
  /** Retrieves a numeric configuration value from Lua config.
   
      @tparam T The numeric type to retrieve (must be integral or floating point)
      @param key The configuration key to look up
      @return The configuration value as type T, or T(0) if not found
  */
  template<typename T>
  static double num(std::string_view key);
  ```

## Best Practices and Standards

### Core Principles
- **KISS (Keep It Simple, Stupid)**: Favor simple, straightforward solutions over complex ones
  - Minimize abstraction layers unless they provide clear value
  - Write code that's easy to understand and debug during competition
- **DRY (Don't Repeat Yourself)**: Avoid code duplication
  - Extract common patterns into reusable functions or classes
  - Use constants and configuration objects instead of magic numbers
- **Performance First**: Optimize for low latency and high throughput
  - Minimize allocations in periodic/execute methods
  - Reuse control request objects (don't create new ones each loop)
  - Avoid blocking operations in periodic code
  - Cache frequently accessed values when appropriate
- **Real-Time Reliability**: Robot code must be deterministic and responsive
  - Keep periodic methods fast (< 20ms target)
  - Avoid complex computations in the main robot loop
  - Use appropriate thread safety when needed

### FRC-Specific Best Practices
- Configure motors once in constructors, not repeatedly in periodic methods
- Use Phoenix 6 signals and status signals efficiently
- Leverage command-based programming patterns for autonomous and teleop
- Test code in simulation before deploying to hardware
- Log important telemetry for debugging and tuning

## Phoenix 6 2026 API Guidelines

### Motor Configuration (TalonFX/Kraken)
Phoenix 6 2026 uses a **builder pattern** for configurations. DO NOT assign values directly to config fields.

**Correct way:**
```cpp
configs::TalonFXConfiguration config = configs::TalonFXConfiguration{}
    .WithCurrentLimits(
        configs::CurrentLimitsConfigs{}
            .WithSupplyCurrentLimit(40_A)
            .WithSupplyCurrentLimitEnable(true)
            .WithStatorCurrentLimit(80_A)
            .WithStatorCurrentLimitEnable(true)
    )
    .WithVoltage(
        configs::VoltageConfigs{}
            .WithPeakForwardVoltage(12_V)
            .WithPeakReverseVoltage(-12_V)
    )
    .WithSlot0(
        configs::Slot0Configs{}
            .WithKP(0.1)
            .WithKI(0.0)
            .WithKD(0.0)
            .WithKV(0.12)
    )
    .WithMotorOutput(
        configs::MotorOutputConfigs{}
            .WithInverted(signals::InvertedValue::CounterClockwise_Positive)
            .WithNeutralMode(signals::NeutralModeValue::Coast)
    );

motor.GetConfigurator().Apply(config);
```

**INCORRECT (old API - will not compile):**
```cpp
config.CurrentLimits.SupplyCurrentLimit = 40;  // DON'T DO THIS
config.Voltage.PeakForwardVoltage = 12;        // DON'T DO THIS
```

### Key Points
- Use `.WithXXX()` methods with user-defined literals (`40_A`, `12_V`, `0.5_s`)
- Chain configuration builders with sub-configs (e.g., `CurrentLimitsConfigs{}`)
- Motor inversion uses enum: `signals::InvertedValue::Clockwise_Positive` or `CounterClockwise_Positive`
- Neutral mode uses enum: `signals::NeutralModeValue::Coast` or `Brake`
