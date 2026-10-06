---
title: "HW1: ROS 2, Transforms, and Gazebo"
---

# HW1: ROS 2, Transforms, and Gazebo

**Status: draft scaffold. This assignment is not yet released.**

## Goal

Read a camera topic to detect a target point or object, transform its coordinates
from the camera frame to the robot base frame, and send motor or joint commands
in Gazebo so the robot reaches the target.

## Environment

Use the shared setup described in [environment/README.md](environment/README.md).
The supported platform, ROS 2 and Gazebo versions, robot model, camera interface,
and launch commands will be specified before release.

## Your implementation

Put your code in [my_solution/](my_solution/). Starter files and TODOs will be added
there before release.

Your implementation will cover:

1. Receiving camera data and identifying the target.
2. Obtaining a target position in the camera coordinate frame.
3. Transforming the target position into the robot base frame using TF2.
4. Sending the robot commands needed to reach the target in simulation.

The exact interfaces, required files, allowed dependencies, and success criteria
remain to be defined. Do not assume pixel coordinates alone give a 3D position;
the assignment will specify how depth or target geometry is provided.

## Public checks

See [tests/README.md](tests/README.md). Check commands and acceptance criteria will
be added with the starter code.

## Deliverables

- Your implementation in `my_solution/`.
- A short report in `my_solution/report.md`, describing your approach and results.
- A demonstration link in the report.

Report requirements and demonstration length remain to be defined.

## Submission

Follow the [repository submission workflow](../../README.md#working-on-an-assignment).
Submit your `hw01` branch through a pull request titled
`HW01 — Firstname Lastname` or, for a group,
`HW01 — Firstname Lastname, Firstname Lastname`.

The deadline and the method for recording the submitted commit hash will be
announced before release.

## Evaluation

The course plan calls for a short report and a 15-minute live Zoom defense per
group. The grading rubric will be published before release.
