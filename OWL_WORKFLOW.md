# OWL (Omnidirectional Wi-Fi Localization) — Workflow & Contribution Guide

This guide explains the branch strategy, development workflow, and upstreaming process for the OWL project on top of PX4-Autopilot.

---

## 1. Repository & Branch Architecture

```
upstream (PX4/PX4-Autopilot)
   │
   ▼  git fetch upstream && git merge --ff-only upstream/main
 origin/main (PeraHive/PX4-Autopilot:main)
   │
   ├──> owl_dev (Active Research & Development Branch)
   │       ├── Experimental commits & tests
   │       ├── Local research notes, thesis logs, scripts
   │       └── Pushed to origin/owl_dev (backed up on GitHub)
   │
   ▼  (When ready to contribute upstream)
feature/owl-localization (Fresh branch off upstream/main)
   │
   ├── Pull ONLY production C++ code & configs from owl_dev
   ├── Clean conventional commits (e.g., feat(estimator): ...)
   └── PR to PX4/PX4-Autopilot:main
```

### Configured Remotes

* **`origin`**: `git@github.com:PeraHive/PX4-Autopilot.git` (Your fork)
* **`upstream`**: `https://github.com/PX4/PX4-Autopilot.git` (Official PX4 repository)

---

## 2. Daily Development Workflow (`owl_dev`)

### Step 1: Work on `owl_dev`

Always ensure you are working on the `owl_dev` branch:

```bash
git checkout owl_dev
```

### Step 2: Commit Freely

You can commit your estimation code, research notes, and test scripts here:

```bash
git add src/modules/owl_localization/
git commit -m "feat(owl): initial estimator implementation"

# Research notes or scripts can also be committed on owl_dev:
git add OWL_NOTES.md
git commit -m "docs: add RSSI triangulation notes"
```


* `feat`: (new feature for the user, not a new feature for build script)
* `fix`: (bug fix for the user, not a fix to a build script)
* `docs`: (changes to the documentation)
* `style`: (formatting, missing semi colons, etc; no production code change)
* `refactor`: (refactoring production code, eg. renaming a variable)
* `test`: (adding missing tests, refactoring tests; no production code change)
* `chore`: (updating grunt tasks etc; no production code change)

### Step 3: Push to Your Fork for Backup

Keep your fork in sync so your work is safe:

```bash
git push origin owl_dev
```

---

## 3. Keeping Up to Date with Upstream PX4

Do this periodically (e.g., weekly or when you need new PX4 features or bug fixes):

### Step 1: Update your local `main`

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

*(If you have already pushed `owl_dev` to GitHub before rebasing, update your remote branch with `git push origin owl_dev --force-with-lease`)*.

---

## 4. Preparing a Clean Pull Request to PX4 Upstream

When your localization algorithm is ready to contribute to official PX4:

### Step 1: Ensure `main` is fresh

```bash
git checkout main
git fetch upstream
git merge --ff-only upstream/main
```

### Step 2: Create a fresh feature branch off `main`

```bash
git checkout -b feature/owl-localization main
```

### Step 3: Bring **only** the production code from `owl_dev`

Do **not** merge `owl_dev` directly. Instead, check out only your module and config files:

```bash
# Pull only the module files:
git checkout owl_dev -- src/modules/owl_localization/

# Pull board configuration or uORB message changes if any:
# git checkout owl_dev -- msg/owl_status.msg
```

*Notice: Any research notes (`OWL_NOTES.md`), scratch scripts, or logs in `owl_dev` will stay behind and NOT touch this branch.*

### Step 4: Run PX4 Style & Format Checks

PX4 CI requires code formatting compliance:

```bash
make format
git diff --stat
```

### Step 5: Commit with PX4 Conventional Commit Format

Format: `type(scope): description`

```bash
git add src/modules/owl_localization/
git commit -m "feat(estimator): add omnidirectional wifi localization module"
```

### Step 6: Push and Open PR

```bash
git push -u origin feature/owl-localization
```

Then visit GitHub to open a Pull Request targeting:

* **Base repository**: `PX4/PX4-Autopilot` (branch: `main`)
* **Head repository**: `PeraHive/PX4-Autopilot` (branch: `feature/owl-localization`)
