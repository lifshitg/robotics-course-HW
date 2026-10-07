"""Public simulator checks; do not import any student or instructor solution."""
import gymnasium as gym
import numpy as np


def test_defaults_and_episode_boundary():
    env = gym.make('Swimmer-v5')
    try:
        observation, _ = env.reset(seed=0)
        assert observation.shape == (8,)
        assert env.action_space.shape == (2,)
        assert np.all(env.action_space.low == -1)
        assert np.all(env.action_space.high == 1)
        assert env.spec.max_episode_steps == 1000
        for i in range(1000):
            observation, reward, terminated, truncated, _ = env.step(np.zeros(2))
            assert np.isfinite(observation).all() and np.isfinite(reward)
            assert not terminated
            assert truncated == (i == 999)
    finally:
        env.close()


def test_reset_and_action_seed_reproducibility():
    a, b = gym.make('Swimmer-v5'), gym.make('Swimmer-v5')
    try:
        np.testing.assert_array_equal(a.reset(seed=11)[0], b.reset(seed=11)[0])
        a.action_space.seed(17)
        b.action_space.seed(17)
        for _ in range(20):
            x, y = a.action_space.sample(), b.action_space.sample()
            np.testing.assert_array_equal(x, y)
            ax, bx = a.step(x), b.step(y)
            np.testing.assert_array_equal(ax[0], bx[0])
            assert ax[1:4] == bx[1:4]
    finally:
        a.close()
        b.close()
