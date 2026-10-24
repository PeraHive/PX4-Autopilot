# OWL (Omnidirectional Wi-Fi Localization) — Workflow & Contribution Guide

This guide aligns your local research workflow with the official [PX4 Contribution Guidelines](CONTRIBUTING.md). It outlines how to conduct active research on your fork without polluting PR branches, and how to prepare submissions that pass PX4 CI and review standards.

---

## 1. Architecture Overview

```
upstream (PX4/PX4-Autopilot:main)
   │
   ▼  git fetch upstream && git merge --ff-only upstream/main
 origin/main (PeraHive/PX4-Autopilot:main)
   │
   ├──> owl_dev (Active Research & Development Branch)
   │       ├── Algorithm implementation & experiments
   │       ├── Research logs, thesis notes, sandbox scripts
   │       └── Backed up to origin/owl_dev
   │
   ▼  (When ready to contribute upstream)
feature/owl-localization (Clean feature branch off fresh upstream/main)
   │
   ├── Cherry-pick/checkout ONLY production C++ code & configs from owl_dev
   ├── Passes code formatting (`make check_format`) & tests (`make tests`)
   ├── Conventional commit: type(scope): description + Assisted-by trailer (if AI-assisted)
   └── Pull Request targeting PX4/PX4-Autopilot:main
```

### Configured Remotes
* **`origin`**: `git@github.com:PeraHive/PX4-Autopilot.git` (Your GitHub fork)
* **`upstream`**: `https://github.com/PX4/PX4-Autopilot.git` (Official PX4 repository)

---

## 2. Daily Research Workflow (`owl_dev`)

### Step 1: Switch to `owl_dev`
```bash
git checkout owl_dev
```

### Step 2: Develop and Commit Freely
In `owl_dev`, you can commit research artifacts, test scripts, and work-in-progress code:
```bash
git add src/modules/owl_localization/
git commit -m "feat(owl): initial Wi-Fi localization module"

git add OWL_RESEARCH_NOTES.md
git commit -m "docs: add triangulation and ranging observations"
```

### Step 3: Back Up to Your Fork
```bash
git push origin owl_dev
```

---

## 3. Keeping in Sync with Upstream PX4

Perform this routine periodically to incorporate upstream bug fixes and improvements:

### Step 1: Update local `main`
```bash
git checkout main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
```

### Step 2: Rebase `owl_dev` onto `main`
```bash
git checkout owl_dev
git rebase main
git submodule update --init --recursive
```
*(If you have already pushed `owl_dev` to GitHub prior to rebasing, update your remote branch using `git push origin owl_dev --force-with-lease`)*.

---

## 4. Contributing Upstream (Strictly Aligned with CONTRIBUTING.md)

When your localization code is validated and ready for submission to `PX4/PX4-Autopilot`:

### Step 1: Sync `main`
```bash
git checkout main
git fetch upstream
git merge --ff-only upstream/main
```

### Step 2: Create a Feature Branch
As required by PX4 guidelines, always branch directly off fresh `main`:
```bash
git checkout -b feature/owl-localization main
```

### Step 3: Extract Only Production Code from `owl_dev`
Do **not** merge `owl_dev` directly. Check out only the production module files and necessary configs:
```bash
# Extract the production module:
git checkout owl_dev -- src/modules/owl_localization/

# If uORB topics or board configs were added/modified:
# git checkout owl_dev -- msg/vehicle_odometry.msg
# git checkout owl_dev -- boards/px4/sitl/default.px4board
```
*Note: Personal research notes, raw flight logs, and scratch Python scripts in `owl_dev` remain isolated and will not enter this branch.*

---

## 5. PX4 Quality, Formatting & Testing Standards

Before committing to your PR branch, verify that all PX4 CI requirements are met:

### A. Code Style & Formatting
PX4 enforces code formatting via astyle. CI will reject PRs with formatting violations.
```bash
# Format C/C++ code automatically:
make format

# Or check if any formatting discrepancies exist:
make check_format
```

### B. Unit & Integration Testing
Per PX4 contribution policy:
* **Unit Tests**: Add tests under `src/modules/owl_localization/test/` if applicable, and verify:
  ```bash
  make tests
  ```
* **SITL Verification**: Test localization in SITL:
  ```bash
  make px4_sitl gz_x500       # Multicopter
  # or
  make px4_sitl gz_r1_rover   # Rover
  ```
* **Flight / Simulation Logs**: Upload generated `.ulg` logs to [Flight Review](https://logs.px4.io) to include in your PR description as verification evidence.

---

## 6. Commit Message & PR Rules

PX4 strictly enforces [Conventional Commits](https://www.conventionalcommits.org/) and PR title conventions.

### Format
```text
type(scope): description

Optional body explaining rationale, math, and architecture.

Assisted-by: Model:version (Required only if AI tools were used)
```

### Valid Types & Scopes
* **Allowed Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`
* **Applicable Scopes**:
  * `ekf2` (if integrating directly into EKF2)
  * `estimator` or `owl_localization` (if adding a standalone module)
  * `simulation` (if updating SITL/Gazebo models)
  * `uorb` (if introducing new uORB topics)

### Example Production Commit:
```bash
git add src/modules/owl_localization/
git commit -m "feat(estimator): add omnidirectional wifi localization module

Implements multilateration state estimation for UAVs and rovers
using omnidirectional Wi-Fi signal ranging.

Assisted-by: Gemini:gemini-3.8-flash"
```

---

## 7. Submitting & Managing the PR

### Step 1: Push Branch to Your Fork
```bash
git push -u origin feature/owl-localization
```

### Step 2: Open Pull Request
Open the PR on GitHub:
* **Base repository**: `PX4/PX4-Autopilot` | **Base branch**: `main`
* **Head repository**: `PeraHive/PX4-Autopilot` | **Compare**: `feature/owl-localization`
* **PR Title**: Must follow `type(scope): description` (e.g., `feat(estimator): add omnidirectional wifi localization module`).
* **PR Description**: Include background, implementation details, SITL/test results, and a link to your [Flight Review](https://logs.px4.io) log.

### Step 3: Addressing Review Feedback
Per PX4 policy, once review has begun:
* Address review feedback by adding **new commits** on `feature/owl-localization`.
* Do **not** amend, squash, or force-push during active review unless explicitly requested for rebasing on `main`.
