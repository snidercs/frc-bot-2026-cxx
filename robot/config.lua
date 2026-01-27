local config = {
    robot_name  = "Indy",   -- Robot name
    team        = 9431,     -- FRC team number
    period      = 20,       -- in milliseconds

    gamepad     = false,     -- Control robot with gamepad

    intake_top_device_id = 14,
    intake_top_can_bus = "rio",
    intake_bottom_device_id = 15,
    intake_bottom_can_bus = "rio"
}

return config
