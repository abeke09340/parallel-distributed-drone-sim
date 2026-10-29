# Contributing & Git Collaboration Guide

This guide defines the collaborative workflow for **Abdirakhman** and **Meiirlan** to develop and maintain the **Distributed Multi-Agent Simulation** repository smoothly without git conflicts or breaking changes.

---

## 👥 Roles & Branch Ownership

| Module / Area | Primary Owner | Secondary Reviewer |
| :--- | :--- | :--- |
| **Simulation Engine & OpenMP Parallelism** | Abdirakhman | Meiirlan |
| **Agent Migration Protocol (MPI Serialization)** | Abdirakhman | Meiirlan |
| **Decentralized Sensor Fusion** | Meiirlan | Abdirakhman |
| **Distributed Task Allocation (Auction/Consensus)** | Meiirlan | Abdirakhman |
| **Fault Tolerance & Heartbeat Manager** | Abdirakhman & Meiirlan | Joint Review |
| **Visualization Bridge (Unity / Socket)** | Meiirlan | Abdirakhman |

---

## 🌿 Git Branching Strategy

We follow a clean **Feature Branch Workflow**:

```
main  ---------------------------------------------------> [Production / Stable Releases]
         \                                       /
feature/migration  -----------------------------' [Pull Request & Code Review]
            \                               /
feature/sensor-fusion  --------------------'
```

1. **`main` is protected:** No direct commits to `main` without testing!
2. **Branch Naming:**
   - Features: `feature/agent-migration`, `feature/sensor-fusion`, `feature/openmp-opt`
   - Fixes: `fix/mpi-ack-timeout`, `fix/drone-boundary-clamp`
   - Docs/Refactoring: `docs/architecture-update`, `refactor/message-types`

---

## 🔄 Daily Workflow Step-by-Step

### 1. Before Starting Work (Sync with Remote)
Always pull the latest changes from `main` before starting any new branch or feature:
```bash
git checkout main
git pull origin main
```

### 2. Create a New Feature Branch
```bash
git checkout -b feature/your-feature-name
```

### 3. Commit Frequently with Conventional Commits
Use standard prefixes for commit messages:
- `feat:` A new feature or capability
- `fix:` A bug fix
- `perf:` Performance improvements (e.g., OpenMP scheduling tuning)
- `refactor:` Code cleanup without changing functionality
- `docs:` Documentation or diagram updates
- `test:` Adding or updating unit/simulation tests

**Example:**
```bash
git commit -m "feat(migration): implement 2-phase ownership ack over MPI"
```

### 4. Keep Your Branch Up to Date
If your teammate merged changes to `main` while you were working:
```bash
git fetch origin
git rebase origin/main
# Or git merge origin/main
```

### 5. Verify Build Locally Before Pushing!
Make sure the project compiles cleanly without warnings:
```bash
cmake -B build
cmake --build build
./scripts/run_local.sh
```

### 6. Push and Create a Pull Request (PR)
```bash
git push -u origin feature/your-feature-name
```
Then open GitHub, create a Pull Request into `main`, request a review from your teammate, and merge once approved!

---

## 🛡 How to Avoid Git Conflicts in C++

1. **Modular Code:** Keep different systems in separate files (`drone.cpp`, `sensor_fusion.cpp`, `task_coordinator.cpp`) rather than one huge `main.cpp`.
2. **Never Commit Build Directories:** The `.gitignore` excludes `build/`, `bin/`, `*.o`, `.DS_Store`. Always check `git status` before `git add .`.
3. **Run Code Formatter:**
   ```bash
   clang-format -i src/**/*.cpp include/**/*.hpp
   ```
4. **Communicate in Chat:** Ping each other ("I am editing `types.hpp` right now") when modifying shared header files.

---

## 🆘 Resolving Merge Conflicts

If a conflict occurs during `git pull` or `git rebase`:
1. Check conflicted files: `git status`
2. Open conflicted files in VS Code / IDE. Look for `<<<<<<< HEAD`, `=======`, `>>>>>>>`.
3. Choose the correct combined code.
4. Stage and finish:
   ```bash
   git add <resolved-file>
   git rebase --continue   # if rebasing
   # or git commit -m "merge: resolve conflicts with main"
   ```
