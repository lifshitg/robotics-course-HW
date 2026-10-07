---
title: "HW4: Policy Gradients in Swimmer"
---

# HW4: Policy Gradients in Swimmer

**Draft: see the [verification record](environment/VERIFICATION.md) before distribution.**

## What you will learn

Implement episodic policy-gradient learning yourself, first using vanilla REINFORCE,
then adding a learned value-function baseline. Compare their behavior rather than
assuming that the more elaborate method always wins. The required experiment is
small and CPU-only. No ROS, Gazebo, GPU, or ready-made RL algorithm is needed.

Complete [installation and the environment demo](environment/README.md) first.
Create your own training implementation in `my_solution/`; the supplied demo only
checks simulator interaction. There is no training scaffold to fill in.

## The Swimmer environment

`Swimmer-v5` is a MuJoCo physics simulation of a three-segment articulated swimmer
in a fluid-like environment. Its task is to move forward along positive X.
Two controlled joints connect its segments. Learning coordinated motion is the
agent's task; the simulator computes motion from its torques and physical model.
Your homework goal is to implement and understand learning, not hand-design a gait.

The default observation is an eight-element vector:

| Index | Meaning |
| --- | --- |
| 0 | Orientation angle of the front segment |
| 1, 2 | Angles of the two articulated joints |
| 3, 4 | Front-tip linear velocities along X and Y |
| 5 | Angular velocity of the front segment |
| 6, 7 | Angular velocities of the two joints |

Angles are in radians; linear/angular velocities are in metres per second and
radians per second. Absolute X/Y positions are excluded from the observation.
Do not add values from `info` to the policy input. Read shapes from the environment
rather than assuming a different MuJoCo environment has the same interface.

The action is two joint torques, each bounded by `[-1, 1]`. The default reward is
forward velocity minus a small control cost:

$$r_t = v_{x,t} - 10^{-4}\|a_t\|_2^2.$$

Forward velocity is displacement over one environment step; the default step
covers 0.04 simulated seconds. Backward movement gives a negative forward term.
Reward is not distance to a target, a survival bonus, or a binary success flag.
Reset applies small random perturbations to generalized positions and velocities.

Use `gymnasium.make("Swimmer-v5")` with all defaults. An episode lasts at most
1,000 steps. In this environment the default end is **truncation** by the time
limit, rather than task termination. Termination means the environment reaches a
terminal state; truncation means an external limit ends the rollout. Handle both.
For this homework, the finite 1,000-step episode defines the training horizon:
stop at either flag, include its final reward, and do not bootstrap beyond it.
This convention differs from learning an infinite-horizon value at a time limit.

The API is an interaction loop:

1. `reset(seed=...)` returns the initial observation and an information dictionary.
2. Choose an action from your policy using the current observation.
3. `step(action)` returns the next observation, reward, `terminated`, `truncated`,
   and `info` (diagnostics such as position and reward components).
4. Continue until either end flag is true; reset before stepping again.
5. Close the environment when finished.

The [official Swimmer documentation](https://gymnasium.farama.org/environments/mujoco/swimmer/)
explains the physical model, observation components, rewards, and defaults.
[Gymnasium basic usage](https://gymnasium.farama.org/introduction/basic_usage/)
explains the API and spaces.

## Part A: Vanilla REINFORCE

Let a trajectory contain observations $s_0,\ldots,s_{T-1}$, sampled actions
$a_0,\ldots,a_{T-1}$, and their following rewards $r_0,\ldots,r_{T-1}$.
A stochastic policy $\pi_\theta(a\mid s)$ is a probability density over actions,
parameterized by a neural network. During training, sample actions; simply using
a deterministic network output is not the REINFORCE estimator.

Use finite-horizon reward-to-go with **$\gamma=1$** in the required experiment:

$$G_t = \sum_{k=t}^{T-1}\gamma^{k-t}r_k.$$

This includes the reward produced by action $a_t$. Later actions cannot cause
rewards that preceded them, so reward-to-go uses only current and future rewards.
Its final element is the final reward, not zero. Optimize the episode loss:

$$L_\pi = -\sum_{t=0}^{T-1}\log\pi_\theta(a_t\mid s_t)G_t.$$

Gradient descent on the negative objective increases the probability of actions
weighted favorably by their returns. Rewards need not be differentiable through
the physics simulator; the gradient comes from the policy's log density.

```text
initialize policy network

for each episode:
    collect trajectory using current policy
    calculate discounted returns G_t
    calculate loss = -sum(log π(a_t | s_t) * G_t)
    backpropagate loss
    update policy
```

The loop above describes the one-episode estimator. For the required experiment,
collect **ten complete episodes under the unchanged policy**, then average their
summed episode losses and perform **one policy update**. Do not reuse old-policy
trajectories in later policy updates. Keep returns separate at episode boundaries.
For batch size $B=10$, the policy loss is
$L_{\mathrm{batch}}=B^{-1}\sum_{i=1}^B L_\pi^{(i)}$.
Batching independent episodes reduces Monte Carlo noise without changing REINFORCE.
With $\gamma=1$, the reward-to-go weights optimize the same finite-horizon total
reward used in evaluation; no additional outer discount factor is needed.

### Why this homework uses an undiscounted horizon

In Swimmer, an initial thrust followed by a stationary folded posture can score
better under heavy discounting than a sustained gait. A value such as 0.99 has an
effective reward horizon of roughly 100 steps, much shorter than the 1,000-step
episode. For this finite-horizon assignment use 1, so late swimming still matters.
The general return formula above also applies to discounted objectives, but changing
$\gamma$ changes the task being optimized. See
[Making Reinforcement Learning Work on Swimmer](https://arxiv.org/pdf/2208.07587).
That paper reports older Swimmer versions and other algorithms; its benchmark
scores are context, not required REINFORCE scores for this homework.

### Continuous actions and probabilities

Choose and document a stochastic policy appropriate for bounded continuous
controls. The log probability is for the **whole two-dimensional action**, not
two unrelated scalar losses. If its components are conditionally independent,
the joint density is their product and its log is the sum of component logs.

Your executed actions must respect the action space, and the probability model
must match how you generated them. Transforming a random variable changes its
density. Sampling an unbounded Gaussian, clipping its action, and evaluating the
ordinary Gaussian density at the clipped value is inconsistent: many samples
collapse onto a boundary. Explain your own bounded-action construction and its
log density. Standard PyTorch distributions and transforms are allowed.

In score-function REINFORCE, sampled actions are treated as fixed observations
when differentiating their log density. Do not accidentally replace this with a
pathwise gradient through the sampled action. The
[PyTorch distributions documentation](https://docs.pytorch.org/docs/stable/distributions.html)
describes sampling and log probabilities. You must write your own policy and loop.

## Part B: REINFORCE with a learned baseline

Add a separate network $V_\phi(s)$ predicting the expected discounted reward-to-go
from a state under the current policy. Construct:

$$A_t=G_t-V_\phi(s_t),\qquad
L_\pi=-\sum_t\log\pi_\theta(a_t\mid s_t)A_t.$$

This compares a sampled outcome with the expected outcome at its state. An action
can have positive absolute return but negative advantage. Because the baseline
does not depend on the sampled action at that state, subtracting it leaves the
expected score-function gradient unchanged under the usual regularity assumptions.
A useful baseline can reduce variance; a poor or lagging prediction may not help.

Before fitting to the current batch, obtain its baseline predictions and
use them to construct its policy weights. Treat these weights as constants during
the policy update: the policy loss must not train the value network. Separately
fit the value network to the batch's Monte Carlo returns (computed separately
for each episode). Average squared error over all collected transitions:

$$L_V=\frac{1}{N}\sum_{i=1}^B\sum_{t=0}^{T_i-1}(V_\phi(s_t^{(i)})-G_t^{(i)})^2,
\qquad N=\sum_{i=1}^B T_i.$$

Use a separate optimizer and **20 value-gradient updates per collected batch**,
fitting the same Monte Carlo targets. Construct the policy weights before these
fits, and do not repeat the policy update on that batch. Explain the difference
between value prediction error and the policy objective. Do not use TD targets,
bootstrapping, GAE, replay buffers, PPO, SAC, or another actor-critic algorithm.

## Library rules

Allowed: Gymnasium and MuJoCo for simulation; NumPy; PyTorch neural-network layers,
autograd, optimizers, distributions and transforms; Matplotlib; standard Python
and ordinary logging utilities. You may use these building blocks, not a library
implementation of an RL algorithm.

Not allowed: Stable-Baselines3, RLlib, copied CleanRL algorithms, TorchRL training
algorithms, pretrained policies, or copied complete REINFORCE implementations.
Implement trajectory collection, discounted returns, losses, optimization,
evaluation, and comparison yourself. Attribute external conceptual references.

## Required experiment protocol

| Setting | Requirement |
| --- | --- |
| Environment | Default `Swimmer-v5`, unchanged reward and observations |
| Training seeds | 0, 1, 2 for each method |
| Budget | 500,000 training steps per method per seed (six runs) |
| Episodes | Default 1,000-step limit; ten fresh episodes per policy update |
| Discount | 1 (finite-horizon undiscounted objective) |
| Compute | CPU; rendering disabled during training |
| Baseline fitting | Separate network/optimizer; 20 Monte Carlo MSE updates per batch |

Choose a small policy architecture and policy optimizer settings, then hold them
fixed across methods and seeds. For a given seed, both methods must begin with
identical policy parameters. Initializing the extra value network must not change
the initial policy or consume the policy-sampling random stream. Seed Python,
NumPy, PyTorch, environment resets and any distribution sampling you use;
document the scheme. Equal seeds do not make later trajectories identical after
policies diverge. Use the same reset sequence across methods for a given seed.

The budget gives 500 training episodes and 50 policy updates per run.
No reward shaping, return/advantage normalization, or entropy bonuses in the
required comparison. Do not tune one method on the final evaluation seeds.
Disclose optimizer settings, value-network settings, action distribution,
initialization, and any numerical safeguards such as gradient clipping; use the
same policy safeguards for both methods. Pilot/debug runs must be identified and
excluded from required-run plots. Report unsuccessful seeds, not only your best.

### Evaluation

Evaluate every policy before training and after its final update. Use a separate
environment, ten episodes each, with reset seeds **30000–30009**. Sample stochastic
actions and use reproducible evaluation randomness separate from training.
Keep the same evaluation sampling seed scheme across methods and initial/final
checks. Disable gradients and parameter updates; evaluation must not advance the
training environment or its random-number state. Evaluation steps do not count
toward the training budget. Use the final checkpoint, not the best test-selected
checkpoint. Optional deterministic evaluation must be labeled separately.

For each episode report **undiscounted** return $\sum_t r_t$. With the required
$\gamma=1$, this equals $G_0$, but not the average of all $G_t$ values or advantages. Compute each trained policy's ten-episode mean and sample standard
deviation. Then compare the three seed-level means, reporting their mean and
sample standard deviation. Three seeds offer limited evidence; do not present
standard-deviation bands as confidence intervals or claim statistical certainty.

### Learning curves and runtime

Log raw training episode return, cumulative environment steps, episode length,
seed, method, and elapsed time. Plot return against cumulative training steps:
show individual seeds and each method's mean with standard-deviation bands across
seeds. If smoothing, disclose the window, apply it consistently, and retain raw
logs. Align curves at corresponding step counts, not wall times. Report final
evaluation separately; a training curve includes exploration and changing policies.

Record CPU model, package versions, thread settings, and total training/evaluation
wall time. Use a small network and avoid rendering. A near-zero initial torque mean can
help avoid an immediate persistent push into joint limits; document your initialization. The budget is deliberately
short; limited or noisy improvement is possible and is not a reason to conceal a
run or change the required budget. Runtime measurements are in the verification
record; they are not a guarantee for every laptop.

## What to submit

Keep your source and report under `assignments/hw04/my_solution/`:

- Training/evaluation code and reproducible commands from the repository root.
- Configuration and lightweight raw logs for all six required runs.
- Learning-curve figures and a table of initial/final evaluation results per seed.
- A short report (approximately 3–5 pages) explaining the policy, returns,
  bounded actions, baseline, fairness, runtime, observed variability, and limitations.
- Setup-demo results and a brief demonstration of a trained swimmer, either live
  rendering or a short recording. The demonstration is illustrative, not evaluation.

Save large checkpoints and temporary output under ignored root `build/hw04/`.
Export the required small figures/logs into your submission directory. Document
how checkpoints can be reproduced. Do not commit a virtual environment or caches.
Follow the [course submission process](../../README.md#submitting-an-assignment).
Use a PR title containing all submitting students' names, for example:

```text
HW04 — Firstname Lastname, Firstname Lastname
```

Pushing alone does not submit the assignment. Include report and demonstration links, and attend the
live defense with every group member.

## Assessment and self-check

There is no minimum reward threshold and the baseline is not required to win.
Assessment covers implementation correctness, fair/reproducible experiments,
clear evidence, and your explanation. Deadlines and weights are announced separately.

Before submitting, check return indexing and final rewards; verify policy gradients
are finite and nonzero; ensure the policy loss cannot update baseline parameters;
check bounded actions and consistent densities; confirm evaluation changes no
parameters or training randomness; confirm all methods receive equal budgets.
Be ready to explain poor results as carefully as successful ones.
