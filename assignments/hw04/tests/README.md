# Public HW4 setup checks

From the repository root, after installing dependencies:

```bash
source .venv/bin/activate
python -m pytest -q assignments/hw04/tests
```

These checks use only the supplied simulator: default dimensions/bounds,
finite observations/rewards, time-limit behavior, and seeded reproducibility.
They neither implement nor grade REINFORCE. Write your own algorithm checks.
