"""Pure decision logic (no ROS): staleness and target consistency."""
from collections import deque
import math

MAX_SIM_AGE_S = 0.5      # observation older than this (simulation time) is stale
MAX_WALL_GAP_S = 2.0     # no fresh camera data for this long (wall time) is stale
MAX_FUTURE_S = 0.05      # tolerate tiny clock skew; anything newer is a bad stamp


def observation_age_problem(stamp_s, now_s, max_age=MAX_SIM_AGE_S, max_future=MAX_FUTURE_S):
    """Return None if the observation is fresh, else a reason string."""
    age = now_s - stamp_s
    if age > max_age:
        return f'stale observation ({age:.2f} s old)'
    if age < -max_future:
        return f'observation stamped {-age:.2f} s in the future'
    return None


class ConsistencyFilter:
    """Accept a target only after N consecutive valid estimates agree.

    Any rejected frame resets the history, so a missing observation can never
    reuse an earlier target.
    """

    def __init__(self, required=3, tolerance_m=0.01):
        self.required = required
        self.tolerance = tolerance_m
        self.points = deque(maxlen=required)

    def reset(self):
        self.points.clear()

    def add(self, p):
        if not all(math.isfinite(c) for c in p):
            self.reset()
            return None
        self.points.append(tuple(p))
        if len(self.points) < self.required:
            return None
        mean = tuple(sum(c) / len(self.points) for c in zip(*self.points))
        if all(math.dist(q, mean) <= self.tolerance for q in self.points):
            return mean
        return None
