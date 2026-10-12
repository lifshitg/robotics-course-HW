# Public HW6 checks

From the repository root, activate the HW6 environment and run:

```bash
source assignments/hw06/environment/.venv/bin/activate
unset PYTHONPATH
python -m pytest -q assignments/hw06/tests
```

Checks cover dataset contracts/checksums, answer separation, frames, grid/ray
geometry, distance units and evaluation. They contain no particle filter or map
update solution. Passing them verifies the supplied infrastructure, not your SLAM.
