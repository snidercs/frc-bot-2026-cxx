# Competition Troubleshooting Checklist

This page should be the calm, fast checklist for match-day problems.

## Before a match

- robot code deployed successfully
- driver station connected
- battery healthy and fully seated
- CAN devices all online
- vision cameras streaming and detecting tags
- turret zero / calibration sanity-checked
- correct autonomous selected

## If the robot will not drive

- check DS enable state
- check joystick/controller mapping
- check CAN status and brownout history
- confirm drivetrain motors are online

## If vision looks wrong

- check camera streams
- check AprilTag detection in PhotonVision
- confirm field layout assumptions
- confirm robot pose is sane before trusting vision

## If turret aim is wrong

- verify zero point
- check soft limits
- verify fused robot pose
- confirm target side / alliance assumptions

## If intake or shooter acts strange

- check motor IDs and bus names
- confirm command bindings
- watch telemetry for velocity and current draw
- look for command conflicts between mechanisms

## Best practice

Write down what happened and what fixed it. The best checklist is the one that
gets better after every event.
