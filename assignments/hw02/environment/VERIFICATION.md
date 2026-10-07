# HW2 verification record

**Draft. A fresh operating-system installation and GUI checks remain release gates.**

## Tested platform

Recorded on 2026-10-06–07, Ubuntu 24.04.5 LTS, x86-64, Python 3.12.3.

| Dependency | Tested version |
| --- | --- |
| ROS distribution | Jazzy |
| Gazebo Sim | 8.15.0 (Harmonic) |
| `ros-jazzy-rclpy` | 7.1.12-1noble.20260902.053513 |
| `ros-jazzy-ros-gz-bridge` | 1.0.24-1noble.20260905.070149 |
| `ros-jazzy-gz-sim-vendor` | 0.0.13-1noble.20260905.073557 |
| `ros-jazzy-gz-plugin-vendor` | 0.0.5-1noble.20260820.145930 |
| NumPy / SciPy / Matplotlib | 1.26.4 / 1.11.4 / 3.6.3 |
| Pandoc / XeLaTeX | 3.1.3 / TeX Live 2023 |

These record the tested combination; apt commands in the setup guide may install
later patch versions. Dependencies available on the host and ROS apt packages
extracted into a temporary prefix were used. This is **not** a clean-OS installation
test. No MoveIt, manufacturer package, Nav2 planner, external model download, or
TA file is required by the public simulation.

## Completed checks

- All three ROS packages build. A second, empty workspace containing only the
  public `environment/src` packages also builds successfully; readiness and both
  examples pass using that workspace.
- Headless simulation publishes the raw map, targets, advancing clock, measured
  pose/velocity, contact monitoring, and `map` → `base_link` TF.
- Both student examples run without an assignment implementation. The map example
  receives a 320 × 240 grid at 0.025 m resolution and writes raw NPZ/PNG files.
  The movement example reaches `(1.5,3)` and returns to the start.
- Nine public geometry tests pass, covering map/world consistency, uninflated
  occupancy, coordinate/height rejection, and direct steering.
- Twelve isolated ROS helper tests pass, covering actual feedback-based stopping,
  invalid requests, client/server concurrency, cancellation including before
  acceptance, unavailable localization, contact latching, stalled time, and
  the wall-time deadline. Fake ROS tests use a separate domain.
- Eight private planner tests pass, covering unknown/boundary inflation,
  full-edge collision checking, corner grazing, known shortest paths,
  disconnected graphs, repeatability, and connectivity of all three inflated rooms.
- Offline reference planning finds routes for seeds 0/1/2 in both solvable rooms.
  The blocked reference exhausts its bounded 2,000/5,000-sample attempts without
  finding a route. The private implementation uses the supplied map and individual
  `go_to` calls.
- In actual Gazebo, abandoning a nonzero command stops the robot through the
  independent 0.5-wall-second watchdog. Moving a pillar into contact and then
  separating it confirms contact reporting remains latched after separation.
  The observer reads contact data before its publication rate limit at every
  physics step.
- The HW2 PDF builds with its setup appendix; figure, command blocks, tables, and
  page breaks have been visually inspected. The shared PDF builder also retains
  the default HW1 entry point.

## Runtime results

Every solvable run used a fresh world and passed the independent observer: arrival
within 0.10 m, stopped for two simulated seconds, and no retained obstacle contact.
No action result was used as a substitute for these measurements.

| Layout | Seed | Final measured error | Configured real-time factor |
| --- | --- | --- | --- |
| `wide` | 0 | 0.0191 m | 1 |
| `wide` | 1 | 0.0185 m | 4 |
| `wide` | 2 | 0.0183 m | 4 |
| `mixed` | 0 | 0.0190 m | 1 |
| `mixed` | 1 | 0.0185 m | 4 |
| `mixed` | 2 | 0.0181 m | 4 |

The fresh `blocked` run (seed 0, factor 4) completed its bounded 2,000/5,000-sample
planning attempts in 37.1 wall seconds and reported `no_path_found`. The independent
observer verified no movement beyond 0.03 m from the start, stopping for two
simulated seconds, and no wall/pillar contact. No movement request was issued.

Factor 4 was set through Gazebo's physics service only for validation; the launch
default is 1. Speed limits remain metres/radians per **simulated** second.
Actual wall throughput depends on CPU load. The full navigation matrix used
default Fast DDS. Optional Cyclone DDS was separately tested with readiness,
successful manual destinations, and rejection of NaN coordinates.

Fast DDS bridge cleanup sometimes reported allocator/datawriter failures on this
host and required forced termination by launch. Cyclone DDS cleaned up the ROS
components successfully. Automated noninteractive Gazebo wrappers sometimes
needed SIGTERM escalation; two wrapper exits left Gazebo child processes that
required an explicit SIGINT. All validation processes have been stopped. The
setup guide documents the shutdown limitation; normal
interactive shutdown/restart remains a release check.

Generated logs and figures are local, ignored artifacts under
`build/hw02/validation/`; they contain instructor reference output. Initial wide
seed 0 has a separate arrival record; subsequent runs have individual logs and
`runtime.json`.

## Remaining release gates

- Start from fresh Ubuntu 24.04 and install **only** the documented dependencies,
  build, run talker/listener, readiness, both student examples and full reference
  demonstrations. The separate public-only workspace verifies build isolation,
  not a clean operating system or dependency installation.
- Test Gazebo GUI and RViz interactively, including late subscription to student
  configuration-space/roadmap/path outputs. Headless physics is tested; GUI,
  graphics drivers, Ubuntu VMs, ARM machines, and native Windows/macOS are untested.
- Review the recorded runtime matrix and recheck it on the clean installation
  before changing assignment status from draft.
- Announce deadlines and grading weights; confirm the instructor solution has a
  private backup and is absent from tracked/staged files before publication.

Follow [public check commands](../tests/README.md). Action success is insufficient:
the observer independently measures arrival, two simulated seconds of stopping,
and the retained Gazebo wall/pillar contact history. Finite PRM failure is not a
general proof that no physical path exists.
