# Riptide MuJoCo scene

`riptide.xml` is the MuJoCo scene loaded by `riptide_mujoco/MujocoSystem`
(the ros2_control hardware seam) when launching with `use_mock_hardware:=false`.

## Contents

| Path | What |
|------|------|
| `riptide.xml` | Generated scene: cube AUV base + Franka Panda arm, torque actuators |
| `assets/` | Panda meshes (from MuJoCo Menagerie) |
| `vendor/panda.xml` | Unmodified Menagerie source model (provenance) |
| `vendor/LICENSE` | Menagerie license (Apache-2.0) |
| `generate_scene.py` | Transforms `vendor/panda.xml` → `riptide.xml` |

## Provenance

The Panda model and meshes come from **MuJoCo Menagerie**
(`franka_emika_panda`, google-deepmind/mujoco_menagerie, Apache-2.0).
`generate_scene.py` applies the Riptide-specific transforms:

- rename arm joints `joint{i}` → `fer_joint{i}` so MuJoCo joint/actuator names
  match the URDF / ros2_control interface names (the SystemInterface maps by name);
- replace the position (`<general>`) actuators with **7 torque `<motor>`
  actuators** — Riptide commands joint torques via the `effort` interface;
- nest the arm under a fixed cube `auv_base` body (Phase 3 swaps the fixed base
  for a `<freejoint/>` to make it floating);
- add a floor and an `ee` site at the hand TCP;
- drop the keyframe (its `ctrl` dimension no longer matches the actuators).

Regenerate after editing `generate_scene.py`:

```bash
cd riptide_description/mujoco && python3 generate_scene.py
```

Validate the model compiles (any environment with the `mujoco` Python module):

```bash
python3 -c "import mujoco; mujoco.MjModel.from_xml_path('riptide.xml'); print('ok')"
```

## Consistency note

The kinematics/inertials here (Menagerie Panda) and in the URDF
(`franka_description` fer) are both the Franka arm but come from different
sources, so small numeric differences are expected. If exact agreement matters
for a model-based controller, drive that controller's dynamics model from one
source of truth.
