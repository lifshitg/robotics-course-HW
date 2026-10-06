# HW1 verification record

**Draft: not approved for student release.** Recorded 2026-10-06.

## Evidence from this development run

| Check | Result |
| --- | --- |
| Shared workspace build | All three packages build with colcon, including a new `/tmp` workspace containing only public source |
| Public helper math tests | 12 passed |
| Public ROS action tests, fake backend | 8 passed: invalid goals, concurrency, cancellation, missing backend and execution failure |
| Private perception and observation/TF tests | 14 passed: reconstruction, missing marker, invalid depth, TF direction, missing transform, stale/future observations and calibration |
| Shared readiness demo | All checks pass for each preset, including measured joint movement and return home |
| Minimal student example | Fresh 320×240 RGB image and completed fixed-point motion pass, also from the public-only workspace |
| Real inactive controller | Deactivated with SwitchController; helper returns failure when backend rejects motion |
| Contact positive control | Raised the platform into the arm in a test world; public observer detects contact and fails |
| Preset 1 reference + independent physics observer | 0.0029 m tip error; 2 simulated seconds held; platform/marker contacts clear |
| Preset 2 reference + independent physics observer | 0.0022 m tip error; 2 simulated seconds held; platform/marker contacts clear |
| Preset 3 reference + independent physics observer | 0.0021 m tip error; 2 simulated seconds held; platform/marker contacts clear |
| Reference privacy | `ta_solution/` ignored; no instructor solution is tracked |
| PDF | Pandoc/XeLaTeX output generated; figure, commands and layout visually inspected |

The physics observer reads Gazebo link poses and `ContactSensorData` directly.
It publishes stamped contact samples even when there is no contact. Missing streams
fail the public checker; action success alone cannot pass it. The standard Gazebo
Contact plugin publishes only nonempty samples, which is why HW1 supplies this
small observer. See the [upstream implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc).

Detailed runtime logs and the generated PDF are local ignored output under
`build/hw01/`. Instructor-specific notes remain in ignored `ta_solution/`.

## Tested versions and graphics

| Component | Version |
| --- | --- |
| Ubuntu | 24.04.5 LTS, amd64 |
| Python | 3.12 |
| ROS | Jazzy |
| rclpy | 7.1.12 |
| Gazebo Sim | 8.15.0 (Harmonic) |
| ros_gz_bridge | 1.0.24 |
| gz_ros2_control | 1.2.20 |
| controller_manager | 4.48.1 |
| joint_trajectory_controller / joint_state_broadcaster | 4.42.1 |
| Successful headless renderer | Mesa 24.0.5, software rendering |
| Pandoc | 3.1.3 |
| XeTeX | 0.999995, TeX Live 2023 |

This execution environment already contained ROS/Gazebo. Missing documented ROS
packages were downloaded from the ROS apt repository and extracted into temporary
prefixes; system packages were not changed. The normal `sudo apt install` procedure
has **not** been exercised on a fresh OS here. This is a build/runtime check on an
existing Ubuntu installation, not clean-install certification.

The available Mesa 25.2.8 renderer crashed before publishing camera data. Tests
succeeded using an isolated temporary Mesa 24.0.5 software-rendering prefix.
A virtual X display alone did not fix the Mesa 25 crash. That temporary prefix is
not part of the repository and is not a student installation dependency. Do not
blindly downgrade graphics libraries; validate the documented launch on the
intended classroom hardware before release.

GUI Gazebo, RViz, accelerated GPU drivers, Ubuntu VMs, ARM machines, and other
operating systems are untested. Docker and native Windows/macOS support are outside
this version. Installation commands install current apt versions; the table records
actual tested versions and does not promise that every later package works.

## Required before release

1. Start with a fresh Ubuntu 24.04 amd64 machine/VM. Install only the dependencies
   in `README.md`, clone the public repository, and build without any instructor
   files or preexisting HW1 overlay.
2. Run ROS talker/listener, GUI launch, camera viewing, readiness and the student
   example. Record package versions and the graphics driver. Also test the documented
   headless launch if students will use it.
3. Run the instructor reference and public observer for all three presets on that
   same installation. Keep readiness and independent position/contact results.
4. Recheck helper failure behavior, the instruction PDF, and the tracked-file list.
   Mark HW1 released only after these gates pass. Record the exact VM/graphics
   configurations that passed; do not generalize them to all hosts.
