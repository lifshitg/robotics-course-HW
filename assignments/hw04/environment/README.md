# HW4 installation and environment guide

Use Ubuntu 24.04 and Python 3.12 as the primary setup. Training runs on CPU without
ROS, Gazebo or graphics. Other operating systems are not verified for this version.
See [verification](VERIFICATION.md) for tested versions and runtime.

## 1. Install and activate

Fork/clone using the course README and create your `hw04` branch. Every command
below runs from the repository root. Deactivate Conda or other virtual environments
first. Do not source ROS workspaces for HW4. Clear inherited `PYTHONPATH` settings
so a previously sourced ROS installation cannot leak into this virtual environment.

```bash
sudo apt update
sudo apt install python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate
unset PYTHONPATH
python -m pip install -r assignments/hw04/environment/requirements.txt
python -m pip install torch==2.10.0 --index-url https://download.pytorch.org/whl/cpu
python -m pip check
```

The separate CPU wheel index avoids installing GPU dependencies on Ubuntu x86-64.
No MuJoCo license or separately downloaded robot model is required. Gymnasium
ships the default environment assets. Pytest is included for public setup checks
and your own checks. Do not use `sudo pip` or install into the system ROS Python.

In **every new terminal**, from the root:

```bash
source .venv/bin/activate
unset PYTHONPATH
python -c 'import sys, gymnasium, mujoco, torch; print(sys.executable); print(gymnasium.__version__, mujoco.__version__, torch.__version__); print("CUDA build:", torch.version.cuda)'
```

Expect the executable inside `.venv`, the pinned versions, and `CUDA build: None`.
Installing a CPU-only build does not by itself guarantee your code avoids other
devices: keep networks and tensors on CPU. A small network often runs faster with
one PyTorch thread; record your thread settings in the report.

## 2. Check the simulator

```bash
python assignments/hw04/my_solution/demo_env.py
python -m pytest -q assignments/hw04/tests
```

Expect observation shape `(8,)`, action shape `(2,)`, bounds `[-1, 1]`, two episodes
of 1,000 steps, `terminated=False`, `truncated=True`, finite returns, and a PASS.
Random return is not a readiness threshold; the swimmer need not move well.
The public checks test simulation and reproducibility, not your RL implementation.

Optional viewing:

```bash
python assignments/hw04/my_solution/demo_env.py --render --episodes 1
```

A window should show the three-segment swimmer under random controls. Close it
with Ctrl+C after observation; train with rendering disabled. Default episodes
represent 40 simulated seconds but headless simulation can run much faster.

## 3. Begin the assignment

Read [the task](../instructions.md) and create your own training/evaluation code
in `my_solution/`. This is ordinary Python; no colcon, ROS nodes or custom model
build is needed. Use `gymnasium.make("Swimmer-v5")` without modifying defaults.
Store checkpoints and temporary results under root `build/hw04/`.
Do a small debugging run before launching the six required experiments; label
pilot results separately. Each required run has 500,000 training steps and batches
of ten full episodes. Follow the task's gamma-1 finite-horizon convention; a short
discount horizon can favor an initial push followed by stopping. Disable rendering and record actual runtime.

## Troubleshooting

| Symptom | Check/action |
| --- | --- |
| `ModuleNotFoundError` | Activate `.venv`; use `python -m pip` with the same interpreter. |
| Environment `Swimmer-v5` missing | Check the pinned Gymnasium version; do not substitute old Gym or v4. |
| MuJoCo unavailable | Install the requirements, including the MuJoCo extra; run `python -m pip check`. |
| Externally managed Python error | Install inside `.venv`, not system Python. |
| Viewer cannot open display | Use the headless command; viewing needs a desktop and working OpenGL. |
| GLFW/OpenGL errors | Check GPU drivers/display forwarding; unset inherited `MUJOCO_GL` overrides before retrying. On Ubuntu desktop, `sudo apt install libgl1 libglfw3` may be needed. |
| IDE viewer fails but terminal works | Use a normal Ubuntu Terminal; Snap IDEs may inherit incompatible graphics paths. |
| Training unexpectedly slow | Disable rendering, reduce thread oversubscription, use a small network; avoid retaining entire training history as autograd graphs. |
| Non-finite losses | Inspect observations, distribution scale/log densities and gradients; do not silently discard failed seeds. |

Stop scripts with Ctrl+C; close environments in cleanup paths. `deactivate` leaves
the virtual environment. Rerun from recorded seeds for a fresh experiment.

## References and PDF

- [Swimmer environment](https://gymnasium.farama.org/environments/mujoco/swimmer/)
- [Gymnasium interaction API](https://gymnasium.farama.org/introduction/basic_usage/)
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- [PyTorch CPU installation](https://pytorch.org/get-started/locally/)
- [PyTorch distributions](https://docs.pytorch.org/docs/stable/distributions.html)

Instructors can build the PDF from the root (students do not need LaTeX):

```bash
sudo apt install pandoc texlive-xetex texlive-fonts-recommended fonts-dejavu-core
python3 tools/build_hw04_pdf.py
```

Output: ignored `build/hw04/hw04.pdf`, with this guide as an appendix.
