local config = {
    robot_name  = "Indy",       -- Robot name
    team        = 9431,         -- FRC team number
    period      = 20,           -- in milliseconds

    gamepad     = false,        -- Control robot with gamepad
   
    heading_button_index = 18,  -- The button index to use for resetting the heading direction.
    
    intake_trigger_index = 18,  -- The button index to engage the intake.

    intake_top_device_id = 14,
    intake_top_can_bus = "rio",
    intake_bottom_device_id = 15,
    intake_bottom_can_bus = "rio",
    intake_voltage = -4.2,  -- Voltage for intake motor (negative = intake direction)

    climber_device_id = 60,
    climber_can_bus = "rio",
    climber_climb_button_index = 5,  -- Button to climb (move down)
    climber_lower_button_index = 6   -- Button to lower (move up)
}

return config
