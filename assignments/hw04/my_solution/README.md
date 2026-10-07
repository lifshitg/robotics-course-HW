# Your HW4 implementation

Run commands from the repository root. Follow the
[setup guide](../environment/README.md), then:

```bash
source .venv/bin/activate
unset PYTHONPATH
python assignments/hw04/my_solution/demo_env.py
python assignments/hw04/my_solution/demo_env.py --render --episodes 1
```

The example runs random controls and prints spaces and returns. It does not learn.
Headless is the default; the second command requires a working desktop/display.

Create your own Python modules here for training, evaluation and plotting. No ROS
package is needed. Use the same virtual environment; keep your entry-point commands
in this README. Read [instructions.md](../instructions.md) before choosing your
experiment settings. Place temporary results/checkpoints under root `build/hw04/`;
copy required small logs, plots and the report into this folder for submission.
