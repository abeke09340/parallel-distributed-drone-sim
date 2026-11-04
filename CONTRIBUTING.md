# Contributing & Git Collaboration Guide

This guide defines the collaborative workflow for the team to develop and maintain the **Distributed Multi-Agent Simulation** repository smoothly.

---

## 👥 Collaborative Model

The project is developed jointly as a single unified team. Rather than rigid role separation, team members collaborate across all modules as tasks arise, ensuring code is reviewed and stays synchronized.

---

## 🌿 Git Strategy & Direct-to-Main Workflow

The team works directly with the `main` branch:

1. **Before Starting Work (Always Sync!):**
   Always pull the latest changes from your teammate before writing or modifying code:
   ```bash
   git pull origin main
   ```
2. **Modular & Clean Changes:**
   Modify only the files relevant to the current task. Avoid touching unrelated files or performing global reformatting.
3. **Verify Build Locally Before Committing:**
   Ensure CMake compiles cleanly without errors:
   ```bash
   cmake -B build
   cmake --build build
   ```
4. **Descriptive English Commit Messages:**
   Use clear conventional commit messages so everyone can easily track what changed:
   - `feat: ...` for new capabilities
   - `fix: ...` for bug fixes
   - `perf: ...` for parallel optimizations
   - `docs: ...` for documentation updates
5. **Pushing Changes:**
   ```bash
   git push origin main
   ```
6. **Resolving Conflicts:**
   Never use `git push --force`. If remote changes conflict, perform `git pull origin main`, resolve conflicts calmly, verify compilation, and commit the resolution.

---

## 🛡 Best Practices in C++

1. **Modular Code:** Keep different systems in separate files (`drone.cpp`, `sensor_fusion.cpp`, `task_coordinator.cpp`) rather than one huge file.
2. **Never Commit Build Artifacts:** The `.gitignore` excludes `build/`, `bin/`, `*.o`, `.DS_Store`. Always check `git status` before `git add .`.
3. **Communicate in Chat:** Inform your teammate when starting work on a shared module.
