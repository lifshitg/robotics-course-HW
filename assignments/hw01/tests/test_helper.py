"""Public math checks for the supplied motion helper, not a perception solution."""
import math
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'environment/src/hw01_sim'))
from hw01_sim.kinematics import HOME, DEMO, LIMITS, forward, inverse, trajectory_samples


@pytest.mark.parametrize('point', [(0.35, -0.15, 0.14), (0.40, 0, 0.14),
                                   (0.35, 0.15, 0.14), (0.20, -0.20, 0.14)])
def test_valid_points_are_reached(point):
    q = inverse('base_link', point)
    assert math.dist(forward(q), point) < 1e-10
    assert all(lo <= a <= hi for a, (lo, hi) in zip(q, LIMITS))


@pytest.mark.parametrize('frame,point', [
    ('camera_optical_frame', (0.3, 0, 0.14)), ('base_link', (1, 0, 0.14)),
    ('base_link', (0.3, 0, 0.5)), ('base_link', (float('nan'), 0, 0.14)),
    ('base_link', (float('inf'), 0, 0.14)), ('base_link', (0.55, 0, 0.14)),
    ('base_link', (0.05, 0, 0.14)),
])
def test_invalid_goals_are_rejected(frame, point):
    with pytest.raises(ValueError):
        inverse(frame, point)


def test_trajectory_is_smooth_and_within_joint_limits():
    samples = list(trajectory_samples(HOME, DEMO))
    assert samples[0][1] == list(HOME)
    assert samples[-1][1] == list(DEMO)
    assert samples[0][2:] == ([0.0, 0.0], [0.0, 0.0])
    assert samples[-1][2:] == ([0.0, 0.0], [0.0, 0.0])
    assert samples[-1][0] == 5.0
    for _, q, velocity, _ in samples:
        assert all(lo <= a <= hi for a, (lo, hi) in zip(q, LIMITS))
        assert all(abs(v) < 2 for v in velocity)
