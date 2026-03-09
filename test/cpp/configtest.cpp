#include <gtest/gtest.h>
#include "config.hpp"

// Test basic config value retrieval
TEST(ConfigTest, GetStringValue) {
    auto robotName = config::str("robot_name");
    EXPECT_EQ(robotName, "Indy");
}

TEST(ConfigTest, GetIntegerValue) {
    auto team = config::integer("team");
    EXPECT_EQ(team, 9431);
    
    auto period = config::integer("period");
    EXPECT_EQ(period, 20);
}

TEST(ConfigTest, GetNumberValue) {
    auto intakeVoltage = config::number("intake_voltage");
    EXPECT_DOUBLE_EQ(intakeVoltage, -4.2);
}

TEST(ConfigTest, GetBooleanValue) {
    auto gamepad = config::boolean("gamepad");
    EXPECT_FALSE(gamepad);
    
    // Test with fallback
    auto nonExistent = config::boolean("does_not_exist", true);
    EXPECT_TRUE(nonExistent);
}

// Test device ID retrieval
TEST(ConfigTest, GetDeviceIds) {
    auto intakeTopId = config::integer("intake_top_device_id");
    EXPECT_EQ(intakeTopId, 14);
    
    auto intakeBottomId = config::integer("intake_bottom_device_id");
    EXPECT_EQ(intakeBottomId, 15);
    
    auto climberId = config::integer("climber_device_id");
    EXPECT_EQ(climberId, 1);
}

// Test CAN bus strings
TEST(ConfigTest, GetCanBusStrings) {
    auto intakeTopBus = config::str("intake_top_can_bus");
    EXPECT_EQ(intakeTopBus, "rio");
    
    auto intakeBottomBus = config::str("intake_bottom_can_bus");
    EXPECT_EQ(intakeBottomBus, "rio");
    
    auto climberBus = config::str("climber_can_bus");
    EXPECT_EQ(climberBus, "rio");
}

// Test button index retrieval
TEST(ConfigTest, GetButtonIndices) {
    auto headingButton = config::integer("heading_button_index");
    EXPECT_EQ(headingButton, 8);
    
    auto climbButton = config::integer("climber_climb_button_index");
    EXPECT_EQ(climbButton, 16);
    
    auto lowerButton = config::integer("climber_lower_button_index");
    EXPECT_EQ(lowerButton, 1);
    
    auto turretAimButton = config::integer("turret_aim_button_index");
    EXPECT_EQ(turretAimButton, 7);
    
    auto turretShootButton = config::integer("turret_shoot_button_index");
    EXPECT_EQ(turretShootButton, 18);
}

// Test missing values return defaults
TEST(ConfigTest, MissingValueDefaults) {
    auto missingInt = config::integer("does_not_exist");
    EXPECT_EQ(missingInt, 0);
    
    auto missingDouble = config::number("also_missing");
    EXPECT_DOUBLE_EQ(missingDouble, 0.0);
    
    auto missingStr = config::str("not_there");
    EXPECT_EQ(missingStr, "");
    
    auto missingBool = config::boolean("nope");
    EXPECT_FALSE(missingBool);
}

// Test generic get function
TEST(ConfigTest, GenericGet) {
    auto teamObj = config::get("team");
    EXPECT_TRUE(teamObj.valid());
    EXPECT_TRUE(teamObj.is<int>());
    EXPECT_EQ(teamObj.as<int>(), 9431);
    
    auto nameObj = config::get("robot_name");
    EXPECT_TRUE(nameObj.valid());
    EXPECT_TRUE(nameObj.is<std::string>());
    EXPECT_EQ(nameObj.as<std::string>(), "Indy");
}

// Test invalid key returns invalid object
TEST(ConfigTest, InvalidKeyReturnsInvalid) {
    auto invalid = config::get("this_key_does_not_exist");
    EXPECT_FALSE(invalid.valid());
}

// Test intake stutter length is a valid duration (>= 0)
TEST(ConfigTest, IntakeStutterLengthIsNonNegative) {
    auto stutterLength = config::number("intake_stutter_length");
    EXPECT_GE(stutterLength, 0.0);
}

// Test type conversion safety
TEST(ConfigTest, TypeConversionSafety) {
    // Trying to get a string as an integer should return 0
    auto wrongType = config::integer("robot_name");
    EXPECT_EQ(wrongType, 0);
    
    // Trying to get an integer as a string should return empty
    auto wrongTypeStr = config::str("team");
    EXPECT_EQ(wrongTypeStr, "");
}
