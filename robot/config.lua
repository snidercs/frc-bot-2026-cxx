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
    intake_bottom_can_bus = "rio"
}

return config
