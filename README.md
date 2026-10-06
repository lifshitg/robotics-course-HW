# Robotics Course Assignments

This is the working repository for the practical assignments in the Robotics &
Autonomous Systems course. It provides assignment instructions, simulation and
development environments, student starter code, and public checks.

Students fork this repository, complete each assignment on a separate branch in
their fork, and submit a pull request (PR) to this repository. The course repository
remains the shared source for assignment materials; submissions are reviewed and
closed without merging student solutions into `main`.

Assignment instructions are maintained as Markdown files here. These files will
also serve as the source for the assignment PDFs, so the online instructions and
PDFs can be generated from the same content.

## Assignments

| Assignment | Topic | Status |
| --- | --- | --- |
| [HW1](assignments/hw01/instructions.md) | ROS 2, transforms, and Gazebo | Scaffold; not yet released |

Additional assignment folders will be added as the course progresses. HW1 currently
contains the folder structure and draft documentation. Its runnable environment,
starter code, public checks, and PDF generation are still to be implemented.

## Repository structure

```text
robotics-course-HW/
├── README.md                  # Repository guide and submission workflow
├── TA_plan.md                 # Course and assignment plan
├── .gitignore                 # Local solutions, caches, and generated output
└── assignments/
    └── hw01/
        ├── instructions.md    # Assignment requirements and PDF source
        ├── assets/            # Instruction figures and diagrams
        ├── environment/       # Shared dependencies and simulation setup
        ├── my_solution/       # Student starter files and submitted work
        └── tests/             # Public checks students can run
```

Each assignment follows the same structure:

- **`instructions.md`:** Read this first for the task, deliverables, and evaluation.
- **`assets/`:** Figures and diagrams referenced by the instructions.
- **`environment/`:** Setup instructions, dependencies, robot models, and simulation
  configuration shared by all implementations of the assignment.
- **`my_solution/`:** Complete the starter files here and add your report and other
  required deliverables. This folder is tracked by Git and included in your PR.
- **`tests/`:** Public checks and their run instructions.

## Forking and initial setup

Do this once for the course:

1. Click **Fork** on [this repository](https://github.com/lifshitg/robotics-course-HW)
   to create your own copy on GitHub. Groups should use one shared fork.
2. Clone your fork to your computer and connect it to the course repository:

   ```bash
   git clone https://github.com/YOUR-USERNAME/robotics-course-HW.git
   cd robotics-course-HW
   git remote add upstream https://github.com/lifshitg/robotics-course-HW.git
   ```

Replace `YOUR-USERNAME` with the fork owner's username. `origin` is your fork;
`upstream` is the course repository where we publish assignment materials.

## Working on an assignment

Keep `main` for course materials. Use a separate branch for each assignment.
Before starting, commit any current work, update `main`, and create your branch:

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
git switch -c hw01
```

For later assignments, replace `hw01` with the assignment number.
Read its `instructions.md`, follow the setup in `environment/`, and complete your
code and report in `my_solution/`. Run the public checks in `tests/`.

Commit and push your work to your fork. For HW1:

```bash
git add assignments/hw01/my_solution/
git commit -m "Complete HW1"
git push -u origin hw01
```

## Submitting an assignment

**Pushing alone is not a submission. Open a pull request (PR) to submit.**

1. On GitHub, open your fork and choose **Compare & pull request** for your branch.
2. Set the destination to `lifshitg/robotics-course-HW`, branch `main`, and the
   source to your fork's assignment branch, such as `hw01`.
3. Title the PR with the assignment and all submitting students' names:
   `HW01 — Firstname Lastname, Firstname Lastname`.
4. Include a brief description, check results, and report/demonstration links.
5. Open the PR as ready for review before the deadline.

Use one PR per assignment per student or group. Push further updates to the same
branch to update that PR. Staff will record the submitted commit at the deadline,
grade it, and close the PR without merging it. Deadlines will be announced per
assignment. Public forks and PRs are visible to other students.

## Instructor development

For local development, each assignment may also have a `ta_solution/` folder:

```text
assignments/hw01/ta_solution/   # Local reference implementation and private notes
```

The root `.gitignore` excludes `assignments/*/ta_solution/`. This folder is not
distributed through the course repository, so students will not receive it when
forking or cloning. Both student and TA implementations must use the same
`environment/` setup and public checks.

Ignored files have no backup in this repository's Git history. Keep a private backup
of TA solutions and never force-add them. Ignoring a file does not remove it from
Git if it was already committed. Before publishing assignment materials, review
the staged diff and confirm that instructor solution files are not tracked.
