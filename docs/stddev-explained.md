# Standard Deviation, in Simple Terms

This page explains what `stdDev` means in our robot code, especially for vision pose updates.

## The short version

Standard deviation is just a way of saying:

> **How much should we trust this measurement?**

- **small std dev** = "this measurement is probably pretty good"
- **large std dev** = "this measurement might be noisy, so trust it less"

In our robot, vision does **not** directly overwrite pose.
Instead, vision gives the drivetrain pose estimator a measurement **plus** a confidence level.
That confidence level is the `stdDevs` array.

## A simple mental model

Imagine three friends trying to tell you where the robot is:

- the gyro says heading
- wheel odometry says how far you drove
- the camera says where it thinks you are on the field

If the camera has a **small std dev**, the estimator treats it like:

> "This camera sounds confident. Listen closely."

If the camera has a **large std dev**, the estimator treats it like:

> "This might still help, but don't let it yank the pose around."

So `stdDev` is basically a **trust knob**.

## What the numbers mean

For vision we usually pass 3 values:

- X std dev
- Y std dev
- theta std dev

That means:

- how uncertain vision is in the field X direction
- how uncertain vision is in the field Y direction
- how uncertain vision is about rotation

In this project, theta is intentionally set very high so vision does **not** fight the gyro for heading.

## Example

If a camera sees:

- multiple tags
- at close range
- with a stable solve

then we use a **smaller** X/Y std dev.

If a camera sees:

- tags that are farther away
- fewer tags
- or measurements that have been jumping around

then we use a **larger** X/Y std dev.

That means the pose estimator will still consider the measurement, but more gently.

## How this shows up in our code

Our vision code builds a `VisionMeasurement` and passes:

- the pose
- the timestamp
- the `stdDevs`

into `AddVisionMeasurement()`.

That means vision is not just saying:

> "The robot is here."

It is saying:

> "The robot is probably here, and I'm this confident about it."

## Why we care

Without std dev tuning:

- bad vision measurements can jerk the pose around
- noisy cameras can fight odometry
- aiming can become unstable

With reasonable std dev tuning:

- good measurements help a lot
- shaky measurements still help a little
- the estimator blends everything more smoothly

## In plain English

A good way to think about it is:

- **reject** = "this measurement is too bad, ignore it"
- **large std dev** = "this measurement is maybe okay, but be careful with it"
- **small std dev** = "this measurement looks solid, trust it more"

So the gates in `VisionIO::processResults()` decide **whether a measurement gets in at all**.
Then the std devs decide **how much weight it gets**.

## What adaptive std dev means

Adaptive std dev means we change the trust dynamically.

If recent vision measurements from one camera have been stable, we keep std dev lower.
If recent measurements have been inconsistent or jumpy, we increase std dev.

So instead of hard-rejecting every questionable measurement, we can say:

> "You can still contribute, just with less influence."

## Rule of thumb

- **lower std dev** = more influence
- **higher std dev** = less influence
- **too low** = vision can be overly aggressive
- **too high** = vision becomes too weak to help much

The goal is not perfection.
The goal is giving the estimator a realistic idea of how trustworthy each measurement is.

## Related files

- `src/vision.hpp` — `computeStdDevs()`
- `src/vision.cpp` — measurement selection and gating
- `docs/vision-pipeline-overview.md` — full vision flow
- `docs/plans/visionsync.md` — notes on Option B / C / E / F behavior
