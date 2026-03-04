local config = {
    robot_name  = "Indy",       -- Robot name
    team        = 9431,         -- FRC team number
    period      = 20,           -- in milliseconds

    gamepad     = false,        -- Control robot with gamepad
    
    auto_default_name = "Backup-Shoot-Left",

    heading_button_index = 8,   -- The button index to use for resetting the heading direction.

    intake_trigger_index = 18,  -- The button index to engage the intake.
    intake_eject_index = 19,    -- The button index to eject (stick 1).
    intake_top_device_id = 14,
    intake_top_can_bus = "rio",
    intake_bottom_device_id = 15,
    intake_bottom_can_bus = "rio",
    intake_voltage = -4.2,  -- Voltage for intake motor (negative = intake direction)

    climber_device_id = 1,
    climber_can_bus = "rio",
    climber_climb_button_index = 16,  -- Button to climb (move down)
    climber_lower_button_index = 17,  -- Button to lower (move up)

    -- Turret shooter configuration
    turret_rotation_device_id = 19,
    turret_rotation_can_bus = "rio",
    turret_rotation_axis_stick = 0,
    turret_rotation_axis_index = 3,
    turret_roation_gain = 0.1,

    turret_shooter_device_id = 16,
    turret_shooter_can_bus = "rio",
    turret_aim_button_index = 7,     -- Button to toggle auto-aim
    turret_shoot_button_index = 18,   -- Button to shoot
    
    turret_uptake_device_id = 21,
    turret_uptake_can_bus = "rio",
    
    -- Vision test configuration
    vision_test_camera = "TestCam",  -- PhotonVision camera name for single-camera testing

    -- Drive input shaping
    drive_deadband        = 0.05,  -- Raw axis deadband threshold [0, 1)
    drive_input_exponent  = 2.5,   -- Exponential curve exponent (1.0 = linear, 2.0 = quadratic)
    rotate_deadband       = 0.05,
    rotate_input_exponent = 1.25
}

return config
