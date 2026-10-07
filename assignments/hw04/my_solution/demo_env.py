"""Random-action simulator check; no learning or policy implementation."""
import argparse
import sys
import gymnasium as gym
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--episodes', type=int, default=2)
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error('--episodes must be positive')
    env = gym.make('Swimmer-v5', render_mode='human' if args.render else None)
    try:
        print('Observation:', env.observation_space)
        print('Action:', env.action_space)
        env.action_space.seed(args.seed)
        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            total = 0.0
            for step in range(1, 1001):
                # Sampling from the action space checks interaction, not learning.
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                if not np.isfinite(observation).all() or not np.isfinite(reward):
                    raise RuntimeError('Non-finite observation or reward')
                total += reward
                if terminated or truncated:
                    break
            else:
                raise RuntimeError('Default episode did not end at 1000 steps')
            print(f'Episode {episode}: steps={step}, return={total:.3f}, '
                  f'terminated={terminated}, truncated={truncated}')
        print('PASS: random-action Swimmer interaction; no learning performed')
    finally:
        env.close()


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ImportError) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
