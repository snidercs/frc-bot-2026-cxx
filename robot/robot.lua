---SPDX-FileCopyrightText: Michael Fisher @mfisher31
---SPDX-License-Identifier: MIT

local class = require('luabot.class')
local TimedRobot = require('wpi.frc.TimedRobot')

---@class Robot : TimedRobot A mock robot to use in testing.
local Robot = class(TimedRobot)

function Robot:robotInit()
end

function Robot:simulationInit()
    print("Robot:simulationInit()")
end

function Robot:autonomousInit()
    print("Robot:autonomousInit()")
end

function Robot:teleopInit()
    print("Robot:teleopInit()")
end

function Robot:disabledInit()
    print("Robot:disabledInit()")
end

function Robot:testInit()
    print("Robot:testInit()")
end

function Robot:disabledPeriodic()
end

function Robot:teleopPeriodic()
end

function Robot:robotPeriodic()
end

local function instantiate()
    local robot = setmetatable({}, Robot)
    TimedRobot.init (robot, 0.02)

    return robot
end
Robot.new = instantiate

return Robot
