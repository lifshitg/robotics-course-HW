# HW6 datasets

Two fixed synthetic datasets from different environments, each with 500 synchronized
scans. Load with the public `hw06_support` package. See the task for sensor noise,
frames, statuses and restrictions on answer access.

- `development/`: `sensors.npz`, `metadata.json`, and evaluation-only `truth.npz`.
- `assessment/`: `sensors.npz` and `metadata.json` only. No answer map or trajectory.

Metadata records SHA-256 checksums for the sensor archives. Data generation and
assessment answers are private. Do not regenerate or modify the measurements.
The development true grid stores 0 for free and 100 for occupied; it is an answer,
not a starting map for SLAM. True trajectory shape is `(500,3)` in the initial frame.
