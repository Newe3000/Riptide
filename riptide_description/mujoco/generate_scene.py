#!/usr/bin/env python3
"""Generate riptide.xml (MuJoCo scene) from the vendored Menagerie panda.xml.

Transforms applied:
  * rename the 7 arm joints joint{i} -> fer_joint{i} so MuJoCo joint names match
    the URDF / ros2_control interface names (the SystemInterface maps by name);
  * replace the position (<general>) actuators with 7 torque <motor> actuators
    named fer_joint{i} (Riptide commands joint torques via the effort interface);
  * nest the arm (link0 subtree) under the cube "auv_base" body;
  * Phase 3: make auv_base a FLOATING base (<freejoint/>) in water --
      - global fluid medium (density/viscosity) + the per-geom ellipsoid fluid
        model give drag + added mass (velocity-dependent), so motion is damped
        like it is underwater;
      - NEUTRAL BUOYANCY is modelled as gravity = 0: the vehicle neither sinks
        nor floats, external disturbances (current/impulses) are the forces of
        interest. (MuJoCo's fluid model does not add Archimedes buoyancy, so a
        higher-fidelity Fossen-style buoyancy+gravity model is a later upgrade.)
  * add a seabed floor + an "ee" site at the hand TCP;
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

# --- Fluid medium (sea water) ------------------------------------------------
WATER_DENSITY = 1000.0     # kg/m^3
WATER_VISCOSITY = 0.0009   # Pa.s

# --- AUV base ("hull" stand-in) ----------------------------------------------
CUBE_SIZE = 0.6            # full edge length [m]
HALF = CUBE_SIZE / 2.0
BASE_MASS = 80.0           # hull mass (buoyancy handled via neutral/zero-g model)
START_Z = 1.2              # spawn height above the seabed [m]
# Solid-box inertia about the CoM (kept simple/diagonal).
BOX_I = BASE_MASS * (CUBE_SIZE**2 + CUBE_SIZE**2) / 12.0

# --- Hull thrusters (Phase 4b: base dynamic positioning) ---------------------
# Each thruster is a body-fixed force actuator applied at a site. The layout is
# FULLY ACTUATED -- it spans all 6 rigid-body DOFs:
#   * 1 surge thruster along body +x  -> forward/backward (Fx),
#   * 4 vertical thrusters along body +z at the top corners -> their common mode
#     is heave (Fz, up/down); front/back differential -> pitch (My); left/right
#     differential -> roll (Mx),
#   * 2 lateral thrusters along body +y at +/-x (CoM height) -> common mode is
#     sway (Fy); front/back differential -> yaw (Mz). Placed at z=0 so they add
#     no roll/pitch/heave coupling.
# A force-along-axis at position r produces wrench [axis; r x axis] per unit ctrl,
# so the BaseThrusterController rebuilds the 6xN allocation matrix from this
# geometry (positions + axes mirrored in riptide_controllers.yaml).
THRUST_LIMIT = 200.0       # per-thruster force clamp [N]
TA = 0.22                  # corner offset of the vertical thrusters [m]
#   name,          position (x, y, z),   force axis (body frame)
THRUSTERS = [
    ("thr_surge",   (-HALF, 0.0, 0.0),   (1.0, 0.0, 0.0)),
    ("thr_vfl",     ( TA,  TA, HALF),    (0.0, 0.0, 1.0)),   # front-left
    ("thr_vfr",     ( TA, -TA, HALF),    (0.0, 0.0, 1.0)),   # front-right
    ("thr_vbl",     (-TA,  TA, HALF),    (0.0, 0.0, 1.0)),   # back-left
    ("thr_vbr",     (-TA, -TA, HALF),    (0.0, 0.0, 1.0)),   # back-right
    ("thr_lat_fwd", ( HALF, 0.0, 0.0),   (0.0, 1.0, 0.0)),   # fwd lateral: +Fy, +Mz
    ("thr_lat_aft", (-HALF, 0.0, 0.0),   (0.0, 1.0, 0.0)),   # aft lateral: +Fy, -Mz
]


def main():
    tree = ET.parse(SRC)
    root = tree.getroot()
    root.set("model", "riptide")

    # --- simulation options + fluid medium ----------------------------------
    opt = root.find("option")
    if opt is None:
        opt = ET.Element("option")
        root.insert(0, opt)
    opt.set("timestep", "0.002")          # 500 Hz, matches controller_manager
    opt.set("integrator", "implicitfast")
    opt.set("density", str(WATER_DENSITY))
    opt.set("viscosity", str(WATER_VISCOSITY))
    opt.set("gravity", "0 0 0")           # neutral buoyancy (see module docstring)

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
    # Hull thruster force actuators (applied at the sites, body-frame axis).
    for tname, _pos, (ax, ay, az) in THRUSTERS:
        ET.SubElement(act, "motor", {
            "name": tname,
            "site": tname,
            "gear": f"{ax} {ay} {az} 0 0 0",
            "ctrlrange": f"-{THRUST_LIMIT} {THRUST_LIMIT}",
            "forcerange": f"-{THRUST_LIMIT} {THRUST_LIMIT}",
        })

    # --- drop keyframe (ctrl dimension changed) -----------------------------
    for kf in root.findall("keyframe"):
        root.remove(kf)

    # --- floating AUV base with the arm on top -------------------------------
    wb = root.find("worldbody")
    link0 = next(b for b in wb.findall("body") if b.get("name") == "link0")
    wb.remove(link0)

    base = ET.Element("body", {"name": "auv_base", "pos": f"0 0 {START_Z}"})
    ET.SubElement(base, "freejoint", {"name": "auv_freejoint"})
    ET.SubElement(base, "inertial", {
        "mass": str(BASE_MASS), "pos": "0 0 0",
        "diaginertia": f"{BOX_I:.4f} {BOX_I:.4f} {BOX_I:.4f}",
    })
    # Ellipsoid fluid model on the hull => buoyancy + added mass + drag.
    ET.SubElement(base, "geom", {
        "name": "auv_base_geom", "type": "box",
        "size": f"{HALF} {HALF} {HALF}", "rgba": "0.10 0.32 0.52 1",
        "fluidshape": "ellipsoid",
    })
    # Thruster attachment sites (visualised as small orange markers).
    for tname, (px, py, pz), _axis in THRUSTERS:
        ET.SubElement(base, "site", {
            "name": tname, "pos": f"{px} {py} {pz}", "size": "0.03",
            "rgba": "1.0 0.55 0.0 1",
        })
    link0.set("pos", f"0 0 {HALF}")   # sit the arm on the cube's top face
    base.append(link0)
    wb.append(base)

    # --- seabed floor + EE site ---------------------------------------------
    ET.SubElement(wb, "geom", {
        "name": "floor", "type": "plane", "size": "10 10 0.1",
        "pos": "0 0 0", "rgba": "0.20 0.22 0.24 1",
    })
    for b in root.iter("body"):
        if b.get("name") == "hand":
            ET.SubElement(b, "site", {
                "name": "ee", "pos": "0 0 0.1034", "size": "0.01",
                "rgba": "1 0 0 1",
            })
            break

    # --- optional fixed-base support -----------------------------------------
    # A weld constraint pinning auv_base to the world, INACTIVE by default so the
    # base floats. MujocoSystem activates it when the `fixed_base` hardware
    # parameter is true, turning the AUV into a fixed-base manipulator
    # (reusability: simulate non-floating bases without regenerating the scene).
    eq = root.find("equality")           # reuse the vendored one (gripper fingers)
    if eq is None:
        eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "weld", {
        "name": "base_weld", "body1": "auv_base", "active": "false",
    })

    ET.indent(tree, space="  ")
    tree.write(OUT, encoding="unicode", xml_declaration=False)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
