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

## Submission and live-demo checklist

- Source, configurations, raw logs for all six runs, and root-directory commands
  to reproduce training, evaluation, and plotting.
- A two-panel training figure: all six unsmoothed seed curves on the left;
  each method's mean ± sample standard deviation across three seeds on the right.
  Use training steps on the X-axis and undiscounted episode return on the Y-axis.
- A final-evaluation figure: three seed-mean points per method, a bar for their
  mean, and error bars for their sample standard deviation. Each point averages
  ten stochastic evaluation episodes with reset seeds 30000–30009 and no learning.
- A table of initial/final evaluation means and sample standard deviations per
  method/seed, aggregate results, and runtime.
- A 3–5-page report explaining decisions, changes and their reasons, settings,
  fairness, results, failures, and limitations. Label optional experiments and
  protocol deviations separately. Include setup-demo results.
- Your own checkpoint viewer and root-directory commands for **both methods**.
  Prepare one identified final training seed per method for live rendering without
  learning or retraining. Keep checkpoints under ignored `build/hw04/`.

See [the full requirements](../instructions.md#what-to-submit) for figure captions
and statistical definitions. Submit a PR titled with all students' names; every
group member attends the live defense. There is no minimum reward required to
pass, and the baseline need not win. Instructor results are context, not targets.
