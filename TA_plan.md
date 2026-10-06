# Robotics & Autonomous Systems: TA Course Plan

## 1. Weekly Tirgul Syllabus (Sessions 1–13)
The sessions are structured to progressively bridge theoretical concepts with practical robotics tools, moving from basic ROS architecture to advanced planning and reinforcement learning.

* **Session 1 (Welcoming & ROS):** Course logistics, ROS 2 node architecture, publishers/subscribers, and basic debugging tools.
* **Session 2 (Coordinate Transforms):** Deep dive into TF2, rotation matrices, quaternions, and broadcasting frames between sensors and robot links.
* **Session 3 (Gazebo & Kinematics):** Gazebo physics setup, URDF robot modeling, forward/inverse kinematics, and applying joint commands.
* **Session 4 (Motion Planning):** Translating theoretical configuration space into code, probabilistic roadmaps (PRM), and executing collision-free trajectories.
* **Session 5 (Classical Planning & TAMP):** Writing symbolic domains/problems and linking discrete high-level actions back to geometric feasibility.
* **Session 6 (MDPs):** Formulating Markov Decision Processes, state transitions, and dynamic programming algorithms (Value/Policy Iteration).
* **Session 7 (Imitation Learning & MuJoCo):** Introduction to the MuJoCo simulator, defining continuous state/action spaces, and basic Behavior Cloning.
* **Session 8 (Q-Learning / Value-Based RL):** RL loop, returns, action values, exploration versus exploitation, and Q-learning with a small Gridworld demo.
* **Session 9 (Policy Gradients / Policy-Based RL):** Neural policies for continuous control, policy-gradient and actor–critic intuition, and PPO/SAC overview with a MuJoCo demo.
* **Session 10 (POMDP 1):** Partial observability, observations, belief states, belief updates, and planning under uncertainty.
* **Session 11 (POMDP 2):** Short recap followed by a detailed walkthrough of the POMDP homework solution, emphasizing belief updates and common mistakes.
* **Session 12 (Filters & SLAM):** Particle filters, motion/measurement updates, and an introduction to Simultaneous Localization and Mapping.
* **Session 13 (Foundation Models & Perception):** Utilizing modern vision-language models for semantic understanding and extracting observations.

---

## 2. Assignments Pipeline

| Assignment | Format | Core Deliverable |
| :--- | :--- | :--- |
| **HW1: ROS, Transforms & Gazebo** | Practical | Read a camera topic to detect a target point/object, transform coordinates from the camera frame to the robot base frame, and send motor/joint commands in Gazebo to reach it. |
| **HW2: Search & Motion Planning** | Practical | Implement a search/motion planning algorithm (such as A* on an occupancy grid or PRM) to find a collision-free path for a robot navigating a maze to a goal state. |
| **HW3: Theoretical Foundations I** | Theory | Written mathematical problem set covering coordinate transformations, forward/inverse kinematics, and Markov Decision Processes (Bellman equations, Value/Policy Iteration). |
| **HW4: RL in MuJoCo** | Practical | Train a continuous-control agent (e.g., PPO or SAC) in a standard MuJoCo environment, reporting training curves and evaluation rewards across multiple seeds. |
| **HW5: Theoretical Foundations II** | Theory | Written problem set on advanced RL (policy gradient theorem, actor-critic variance reduction) and POMDPs (Bayesian belief updates, alpha-vectors, policy trees). |
| **HW6: SLAM with PF** | Practical | Implement a Particle Filter for 2D mobile robot localization and occupancy-grid SLAM in a simulation environment with noisy sensor readings. |

---

## 3. Grading & Evaluation Strategy

Because of the widespread use of LLMs and AI tools, evaluation of the practical assignments will heavily rely on live Zoom defenses per group to verify true comprehension and ownership.

### Practical Assignments (HW1, HW2, HW4, HW6)
Evaluated via **15-minute live Zoom defenses** per group and a **small report** including a demonstration of their work and final results. The live defenses are meant to ensure the code works and to allow for questions about the algorithm they have implemented to measure true understanding.

### Theoretical Assignments (HW3, HW5)
The theoretical assignments will be graded on a **binary Pass/Fail basis**. Every student must submit a solution to each theoretical assignment. Submissions will not be checked in depth or graded with a detailed rubric; they will receive a brief review to confirm that the solution is appropriate, addresses the assigned questions, and demonstrates a reasonable good-faith effort. A submission that meets these requirements will receive a Pass; missing, clearly incomplete, irrelevant, or low-effort submissions will receive a Fail.
