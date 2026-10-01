# Riptide — Conan 2 tooling

Conan 2 runs **alongside** colcon/ament (see `docs/plan/`, Step 02). Nothing here
edits a package yet; it freezes the ABI contract and the dependency policy so the
first-party cores (Step 06+) build as binaries that stay link-compatible with the
apt ROS 2 Jazzy stack they are `dlopen`'d beside in one `controller_manager`.

## Install

```bash
python3 -m pip install --user 'conan>=2.0,<3'   # add --break-system-packages on PEP-668 distros
export PATH="$HOME/.local/bin:$PATH"
```

## The ABI profile (the load-bearing part)

`profiles/riptide-linux-release` (host) + `profiles/riptide-linux-build` (build,
two-profile mode) are **frozen from this box**, not auto-detected:

| Setting | Value | Verified from |
|---|---|---|
| `os` / `arch` | Linux / x86_64 | `uname -m` (Ubuntu 24.04) |
| `compiler` / `.version` | gcc / 13 | `gcc -dumpversion` = 13.3.0 |
| `compiler.libcxx` | libstdc++11 | `nm -DC /opt/ros/jazzy/lib/librclcpp.so \| grep __cxx11` = 315 hits → `_GLIBCXX_USE_CXX11_ABI=1` |
| `compiler.cppstd` | gnu17 | C++17 + GNU extensions (ament's `gnu++17` default) |
| `build_type` | Release | `-O3 -DNDEBUG` |

**Do not add `-march=native`/`-mavx`.** It raises Eigen's `EIGEN_MAX_ALIGN_BYTES`
and would corrupt Eigen objects passed across the Conan↔apt `.so` boundary. Keep
the generic 16-byte alignment uniform across every translation unit.

Use it for every first-party `conan` call:

```bash
conan install . -pr:h conan/profiles/riptide-linux-release \
                -pr:b conan/profiles/riptide-linux-build --build=missing
conan profile show -pr:h conan/profiles/riptide-linux-release   # prints the frozen ABI
```

## Remotes

- **conancenter** — `https://center2.conan.io` (the current index; *not* the frozen
  `center.conan.io`). Enabled by default.
- **Private remote** — for first-party + custom (mujoco) packages. Interim: a local
  `conan_server`; target: Artifactory CE / Cloudsmith. Wire it with:

  ```bash
  conan remote add riptide-private https://<your-artifactory>/artifactory/api/conan/riptide
  conan remote login riptide-private <user>
  conan upload  "eigen/3.4.0"   -r riptide-private --confirm     # upload
  conan download "eigen/3.4.0"  -r riptide-private               # download
  ```

  The **upload/download round-trip mechanic** is proven offline here with
  `conan cache save` → `conan remove` → `conan cache restore` → offline re-consume
  (a built package leaves the box as an archive and is re-consumed on a clean
  cache). Point the same package at `riptide-private` for the real HTTP remote.

## Dependency pins & policy

- **Eigen — `eigen/3.4.0`**, pinned to the exact system `libeigen3-dev`
  (`3.4.0-4build0.1`) so Conan Eigen == apt Eigen (no header ODR/alignment skew).
  Never a range or `latest`. `conan/checks/eigen_smoke` proves it resolves and that
  CMakeDeps emits the unchanged `Eigen3::Eigen` target the repo already links.

- **Boost coexistence policy** (decided here; enforced in later steps):
  - Conan `pinocchio/3.8.0` pulls `boost>=1.84`; apt ROS Jazzy ships Boost **1.83**.
    Both live in one process (Conan `libpinocchio` + apt `tf2`/others).
  - **Lock** the Boost version in `pinocchio`'s `package_id` (Step 04): a Boost bump
    forces a pinocchio rebuild, never a silent ABI change.
  - **Contain** Boost inside `libpinocchio.so`: Boost symbols must not cross the
    `.so` boundary the ROS controllers resolve — verified by `nm` in Step 08
    (prefer static/hidden-visibility Boost).
  - **No first-party core** (`riptide_dynamics`, `riptide_control_core`,
    `riptide_geometry`) may expose Boost in its public API. Boost stays a private
    transitive of pinocchio only.

## Recipes

### `mujoco/3.10.0` — `conan/recipes/mujoco/`

Repackages the upstream **prebuilt** Linux SDK (no compiler build). Add a new
version by appending a `sources:` entry (url + sha256) to `conandata.yml`.

```bash
conan create conan/recipes/mujoco --version=3.10.0 \
  -pr:h conan/profiles/riptide-linux-release -pr:b conan/profiles/riptide-linux-build
conan upload "mujoco/3.10.0" -r riptide-private --confirm      # to the private remote
```

- Package id keyed on **os + arch + version only** (prebuilt C-API blob) — one
  binary per platform; a consumer's compiler/std/build_type don't fork it.
- Exports **`mujoco::mujoco`** via CMakeDeps, matching `find_package(mujoco)` (the
  target `riptide_mujoco` adopts in Step 08, replacing `MUJOCO_ROOT`).
- The `libmujoco.so -> libmujoco.so.3.10.0` symlink chain is preserved; `ldd` on
  the test binary resolves `libmujoco.so.3.10.0`.
- **glibc floor: `GLIBC_2.27`** — minimum runner ≈ Ubuntu 18.04 / manylinux_2_27.
- License: Apache-2.0 (staged into `licenses/`).

## Dependency evaluation — `pinocchio/3.8.0` (pinned; conan-center; **not** forked)

Consumed from conan-center — **no recipe authored or forked**. Verified against
this box (Step 04):

- **conan-center availability:** `pinocchio/3.8.0` (the only version), `urdfdom/4.0.0`.
- **Options that fit:** `with_collision_support=False` (the default — no `coal`;
  Riptide computes no collision) and the Python interface off (default).
- **Resolved graph** (`conan graph info`, collision off), all from conan-center
  except the pin:
  - `eigen/3.4.0` — matches the system `libeigen3-dev` (Step 02 pin), header-only.
  - **`boost/1.89.0`** — a *single* version from `[>=1.84 <1.90]` (vs apt ROS 1.83);
    **prebuilt** (Download), so no Boost source build.
  - `urdfdom/4.0.0` → `console_bridge/1.0.2`, `urdfdom_headers/1.1.1`, `tinyxml2/10.0.0`.
  - Only `pinocchio` builds from source; everything else downloads prebuilt.
- **Version delta:** conan-center is **3.8.0**; the from-source `~/.local` build is
  **4.1.0**. The calls `PinocchioModel` makes (`urdf::buildModel`, `crba`,
  `nonLinearEffects`, `forwardKinematics`/`updateFramePlacements`,
  `computeJointJacobians`, `getFrameJacobian` @ `LOCAL_WORLD_ALIGNED`) are the
  stable Pinocchio-3 API; `conan/checks/pinocchio_smoke` compiles+runs them against
  3.8.0 as the behavioral spike. **No API delta found → no fork.**
- **Do not** build a `~/.local` bridge recipe (non-relocatable absolute RPATHs;
  portable only to a box that already has `PINOCCHIO_ROOT`).

Consume it (collision off, eigen pinned):

```bash
conan install --requires=pinocchio/3.8.0 --requires=eigen/3.4.0 \
  -o "pinocchio/*:with_collision_support=False" \
  -pr:h conan/profiles/riptide-linux-release -pr:b conan/profiles/riptide-linux-build --build=missing
conan upload "pinocchio/3.8.0" -r riptide-private --confirm    # to the private remote
```

Only `pinocchio` builds from source (boost/urdfdom are prebuilt downloads). Its
`algorithm/*.cpp` files are RAM-heavy at `-O3` (~2.5 GB each); on a box with less
than ~3 GB RAM per core, cap parallelism or it gets OOM-killed:
`-c tools.build:jobs=2`. Build it once, then `upload` so consumers just download.

**Boost containment finding** (feeds Step 08): `libpinocchio_default.so.3.8.0` has
**no `libboost*.so` runtime dependency** — Boost is linked static/hidden
(`-fvisibility=hidden`). However `nm -D` still shows **13 exported Boost symbols**:
12 weak (vague-linkage) RTTI/vtable entries for `boost::exception` / `boost::bad_get`
/ `boost::wrapexcept` (from `boost::variant`/`get`), and one `STB_GNU_UNIQUE`
`boost::serialization::singleton_module` lock. These are the residual ODR surface
where Conan Boost 1.89 meets apt Boost 1.83 in one `controller_manager`. Risk is
**low** (boost exception / serialization-singleton ABI is stable across 1.83↔1.89)
but must be confirmed by the **Step 08 mixed-process canary**; if it ever misbehaves,
strip these with a linker version-script / `--exclude-libs` on the L3 wrapper.

## Checks

```bash
# ABI + eigen pin + unchanged target, end to end:
conan build conan/checks/eigen_smoke \
  -pr:h conan/profiles/riptide-linux-release -pr:b conan/profiles/riptide-linux-build --build=missing
conan/checks/eigen_smoke/build/Release/eigen_smoke        # prints "eigen ok: x = [0.500 1.000 1.500]"
```
