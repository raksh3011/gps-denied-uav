# Git Workflow

## Branch model

```
main
  |
develop
  |
feature/localization
feature/world-model
feature/planning
feature/mission-safety
```

- `main`: always deployable/demoable. Only receives merges from `develop`.
- `develop`: integration branch. Must always build and pass tests (`colcon build` + `pytest tests/`). Broken `develop` blocks everyone — fix immediately or revert.
- `feature/*`: one per ownership area (see [TEAM_OWNERSHIP.md](TEAM_OWNERSHIP.md)). Branch off `develop`, merge back to `develop`.

## Rules

1. **Nobody pushes directly to `main`.** Not even a "quick fix."
2. **Every change goes through a Pull Request** into `develop` (or `develop` -> `main` for releases).
3. **Every PR requires at least one review** from a teammate other than the author.
4. **Every PR must build successfully** — CI runs `colcon build` on the PR branch.
5. **Every PR must pass tests** — CI runs `pytest tests/` (unit + contract + integration).
6. **Keep PRs small.** One logical change. A PR that touches 5 modules is a sign it should have been split, or that an interface change wasn't isolated first.
7. **Interface changes require team approval.** Any PR touching `src/interfaces/uav_interfaces/msg/*.msg` must tag all four members as reviewers, per [INTERFACES.md](INTERFACES.md#adding-or-changing-an-interface).
8. **Merge into `develop` only after review + green CI.** Squash or regular merge is fine; avoid force-pushing shared branches.
9. **The integration branch must remain functional at all times** — if your merge breaks `develop`, that's the top priority to fix, ahead of new feature work.

## Day-to-day flow

```bash
git checkout develop
git pull
git checkout -b feature/<area>/<short-description>
# ... work, commit ...
git push -u origin feature/<area>/<short-description>
# open PR into develop via GitHub
```

After review + CI green:

```bash
# merge via GitHub UI (squash or merge commit, team preference)
git checkout develop
git pull
git branch -d feature/<area>/<short-description>
```

## Commit messages

Short imperative summary line, body explains *why* if not obvious. Reference the interface/module touched, e.g. `planning: fix trajectory yaw wrap in mock planner`.

## Releases

`develop -> main` merges happen at milestones (e.g. "Milestone 1: mocked pipeline working end-to-end"), tagged `vX.Y`.
