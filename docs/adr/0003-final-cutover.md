# ADR 0003 — Final cutover: Conan-first cores, colcon as a consumer

- **Status:** Accepted (migration Step 10 — `docs/plan/10_consumer-docs-standalone-example-end-state.txt`)
- **Scope:** record the end state of the multi-Conan-package migration and the decisions
  that close it — how the cores are built/consumed, and whether the ament dual-build is
  retired.

## Context

Steps 1–9 took Riptide from a colcon/ament monorepo to a graph of Conan packages:
`riptide_geometry`, `riptide_dynamics`, `riptide_control_core` (L1 first-party), built on
Conan `eigen` / `pinocchio` / custom `mujoco` (L0), with the ROS/`ros2_control` + MuJoCo
sim consuming them through the Step-8 colcon↔Conan seam. Step 8 proved the mixed process
(single Pinocchio 3.8.0, Boost static-in-Pinocchio, no ODR), Step 9 made it reproducible
(lockfile) and CI-published, and this step adds a zero-ROS consumer as the payoff.

## Decision — Conan-first, colcon consumes prebuilt binaries

1. **The L1 cores are Conan-first.** Their source of truth is the Conan recipe; they are
   built with `conan create` and consumed as **prebuilt binaries**. There is no ament
   build of a core.
2. **colcon is a consumer, not a builder, of the cores.** `riptide_control` /
   `riptide_mujoco` resolve the cores + third-party libs via CMakeDeps on
   `CMAKE_PREFIX_PATH` (ADR 0002). They never rebuild a core.
3. **Editable mode is for active core development only.** A dev edits a core and
   `conan editable add`s it for a tight inner loop; the default (and CI) path is the
   cache/remote, pinned by `conan/riptide.lock`.
4. **The standalone example is the payoff and the guard.** `examples/standalone_control_loop/`
   runs a control loop with no ROS / no `~/.mujoco` / no `~/.local/pinocchio`, built in a
   ROS-free CI container so the pure-Conan path cannot rot.

## Decision — shed ament from the L1 cores

The dual-build (ament path) in the cores was a migration scaffold to keep colcon + the sim
green at every step. With Step 8 proven and the full sim passing (the headless smoke is
green in CI), it is retired:

- `riptide_geometry`, `riptide_control_core` — never had a `package.xml`; Conan-only since
  Step 7.
- `riptide_dynamics` — its `package.xml` and the dual-mode `PINOCCHIO_ROOT`/ament branch of
  its `CMakeLists.txt` are **removed** (Step 8 dropped the branch; Step 10 removes the
  `package.xml`). A `COLCON_IGNORE` stays as an explicit guard so colcon never tries to
  treat the directory as a package again.

This is the "optional last move" the plan gated behind a green Step 8 + full sim. It is
safe because no colcon package `ament`-depends on a core any more (verified: no
`package.xml` lists `riptide_dynamics`/`_geometry`/`_control_core`).

## Consequences

- One build system owns each thing: Conan builds the cores; colcon builds only the ROS
  glue (`riptide_control`, `riptide_mujoco`) + the apps/msgs.
- `riptide_msgs` deliberately **stays** rosidl/colcon (L2) — it is a ROS interface package,
  not a reusable core.
- External users get the cores with zero ROS (`docs/CONSUMING.md`); the ROS team gets the
  same binaries through the seam. Both are exercised in CI.
- Reverting a core to a colcon build would mean re-adding a `package.xml` + an ament
  branch — a deliberate act, not an accident.
