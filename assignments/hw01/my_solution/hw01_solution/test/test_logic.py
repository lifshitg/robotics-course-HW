import math

from hw01_solution.logic import ConsistencyFilter, observation_age_problem


def test_fresh_stale_and_future():
    assert observation_age_problem(10.0, 10.2) is None
    assert 'stale' in observation_age_problem(10.0, 10.6)
    assert 'future' in observation_age_problem(10.0, 9.0)


def test_requires_consecutive_agreement():
    f = ConsistencyFilter(required=3, tolerance_m=0.01)
    assert f.add((0.3, 0.1, 0.06)) is None
    assert f.add((0.301, 0.1, 0.06)) is None
    mean = f.add((0.299, 0.1, 0.06))
    assert mean is not None and math.isclose(mean[0], 0.3, abs_tol=1e-9)


def test_disagreement_blocks_target():
    f = ConsistencyFilter(required=3, tolerance_m=0.01)
    for p in [(0.3, 0.1, 0.06), (0.3, 0.1, 0.06), (0.4, 0.1, 0.06)]:
        result = f.add(p)
    assert result is None


def test_reset_prevents_reuse_of_old_target():
    f = ConsistencyFilter(required=2)
    f.add((0.3, 0.1, 0.06))
    f.reset()                      # e.g. a frame without the marker
    assert f.add((0.3, 0.1, 0.06)) is None


def test_nonfinite_resets():
    f = ConsistencyFilter(required=2)
    f.add((0.3, 0.1, 0.06))
    assert f.add((float('nan'), 0.1, 0.06)) is None
    assert f.add((0.3, 0.1, 0.06)) is None
