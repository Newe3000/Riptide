#!/usr/bin/env python3
"""Generate riptide.xml (MuJoCo scene) from the vendored Menagerie panda.xml.

Transforms applied:
  * rename the 7 arm joints joint{i} -> fer_joint{i} so MuJoCo joint names match
    the URDF / ros2_control interface names (the SystemInterface maps by name);
  * replace the position (<general>) actuators with 7 torque <motor> actuators
    named fer_joint{i} (Riptide commands joint torques via the effort interface);
  * nest the arm (link0 subtree) under a fixed cube "auv_base" body, so the scene
    matches the URDF assembly and is ready for Phase 3 (swap the fixed base for a
    free joint to make it floating);
  * add a floor + an "ee" site at the hand TCP;
  * drop the keyframe (its ctrl dimension no longer matches the new actuators).

Run:  python3 generate_scene.py   (regenerates riptide.xml in place)
Provenance: vendor/panda.xml, vendor/LICENSE (mujoco_menagerie, Apache-2.0).
"""
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "vendor", "panda.xml")
OUT = os.path.join(HERE, "riptide.xml")

# Franka torque limits [Nm]: joints 1-4 -> 87, joints 5-7 -> 12.
TORQUE_LIMITS = [87, 87, 87, 87, 12, 12, 12]

CUBE_SIZE = 0.5          # full edge length [m]
CUBE_MASS = 80.0         # [kg]
HALF = CUBE_SIZE / 2.0
# Solid box inertia about center: I = m*(a^2 + a^2)/12 with a = CUBE_SIZE.
BOX_I = CUBE_MASS * (CUBE_SIZE**2 + CUBE_SIZE**2) / 12.0


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    root.set("model", "riptide")

    # --- simulation options -------------------------------------------------
    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option")
        root.insert(0, opt)
    opt.set("timestep", "0.002")          # 500 Hz, matches controller_manager
    opt.set("integrator", "implicitfast")

    # --- rename arm joints joint{i} -> fer_joint{i} -------------------------
    rename = {f"joint{i}": f"fer_joint{i}" for i in range(1, 8)}
    for j in root.iter("joint"):
        if j.get("name") in rename:
            j.set("name", rename[j.get("name")])

    # --- replace actuators with torque motors -------------------------------
    for act in root.findall("actuator"):
        root.remove(act)
    act = ET.SubElement(root, "actuator")
    for i, lim in enumerate(TORQUE_LIMITS, start=1):
        ET.SubElement(act, "motor", {
            "name": f"fer_joint{i}",
            "joint": f"fer_joint{i}",
            "gear": "1",
            "ctrlrange": f"-{lim} {lim}",
            "forcerange": f"-{lim} {lim}",
        })

    # --- drop keyframe (ctrl dimension changed) -----------------------------
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # --- nest the arm under a fixed cube base --------------------------------
    wb = root.find("worldbody")
    link0 = next(b for b in wb.findall("body") if b.get("name") == "link0")
    wb.remove(link0)

    base = ET.Element("body", {"name": "auv_base", "pos": f"0 0 {HALF}"})
    # Phase 3: add <freejoint/> here to make the base floating.
    ET.SubElement(base, "inertial", {
        "mass": str(CUBE_MASS), "pos": "0 0 0",
        "diaginertia": f"{BOX_I:.4f} {BOX_I:.4f} {BOX_I:.4f}",
    })
    ET.SubElement(base, "geom", {
        "name": "auv_base_geom", "type": "box",
        "size": f"{HALF} {HALF} {HALF}", "rgba": "0.10 0.32 0.52 1",
    })
    link0.set("pos", f"0 0 {HALF}")   # sit the arm on the cube's top face
    base.append(link0)
    wb.append(base)

    # --- floor + EE site -----------------------------------------------------
    ET.SubElement(wb, "geom", {
        "name": "floor", "type": "plane", "size": "5 5 0.1",
        "pos": "0 0 0", "rgba": "0.25 0.26 0.28 1",
    })
    for b in root.iter("body"):
        if b.get("name") == "hand":
            ET.SubElement(b, "site", {
                "name": "ee", "pos": "0 0 0.1034", "size": "0.01",
                "rgba": "1 0 0 1",
            })
            break

    ET.indent(tree, space="  ")
    tree.write(OUT, encoding="unicode", xml_declaration=False)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
