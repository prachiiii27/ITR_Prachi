"""
scene.py - builds the MuJoCo world in code (robot + table + cube + tray).

We use mujoco.MjSpec, which lets us load the HEAL XML and then CHANGE and ADD
things from Python:
  - swap the HEAL's torque motors for position servos (like the Panda's),
  - add a two-finger gripper (the HEAL model has none) and a TCP site,
  - a table, a cube with a free joint, a tray made of 5 boxes, lights, cameras.  Because the sizes come from
config.py, changing the table there automatically changes the scene.

    build_model(cube=True,  table=True)  -> full scene used for simulation
    build_model(cube=False, table=True)  -> kinematic model used by the IK
                                            (needs the table/tray for
                                            collision checks, not the cube)
    build_model(cube=False, table=False) -> bare robot (workspace analysis)
"""
import mujoco
import numpy as np
import config as C

BOX = mujoco.mjtGeom.mjGEOM_BOX


def _box(parent, name, pos, size, rgba, **kw):
    return parent.add_geom(name=name, type=BOX, pos=list(pos), size=list(size),
                           rgba=list(rgba), **kw)


def tray_geoms():
    """Return (name, centre, half-size) for the tray floor and 4 walls."""
    tx, ty = C.TRAY_POS
    top = C.TABLE_H
    a, t, h = C.TRAY_INNER, C.TRAY_WALL_T, C.TRAY_WALL_H
    return [
        ("tray_floor", (tx, ty, top + 0.002), (a + t, a + t, 0.002)),
        ("tray_wall_px", (tx + a + t / 2, ty, top + h / 2), (t / 2, a + t, h / 2)),
        ("tray_wall_nx", (tx - a - t / 2, ty, top + h / 2), (t / 2, a + t, h / 2)),
        ("tray_wall_py", (tx, ty + a + t / 2, top + h / 2), (a, t / 2, h / 2)),
        ("tray_wall_ny", (tx, ty - a - t / 2, top + h / 2), (a, t / 2, h / 2)),
    ]


def add_gripper(spec):
    """Bolt a simple parallel-jaw gripper onto the HEAL flange.
    end_effector frame: +z points out of the flange (straight down at home).
      palm   : a box on the flange
      fingers: two boxes on slide joints along local y (they open/close)
      tcp    : site between the finger pads, 1 cm above the finger tips"""
    ee = spec.body(C.EE_BODY)
    palm = ee.add_body(name="gripper_palm", pos=[0, 0, C.FLANGE_Z])
    _box(palm, "palm_geom", (0, 0, C.PALM_H / 2), (0.020, 0.060, C.PALM_H / 2),
         (0.25, 0.25, 0.28, 1), mass=0.3)
    zc = C.PALM_H + C.FINGER_LEN / 2
    for name, sgn in (("finger_left", 1), ("finger_right", -1)):
        f = palm.add_body(name=name, pos=[0, 0, 0])
        f.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, sgn, 0],
                    range=[0, C.FINGER_OPEN], damping=5.0, armature=0.01)
        # inner face of the pad sits exactly at y = +-(joint value)
        _box(f, name + "_pad", (0, sgn * 0.006, zc), (0.012, 0.006, C.FINGER_LEN / 2),
             (0.15, 0.15, 0.15, 1), mass=0.05, friction=[1.5, 0.05, 0.001], condim=4)
    # both fingers move together (finger_right = finger_left)
    spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name1="finger_left",
                      name2="finger_right", data=[0, 1, 0, 0, 0] + [0] * 6)
    act = spec.add_actuator(name="gripper", target="finger_left",
                            trntype=mujoco.mjtTrn.mjTRN_JOINT)
    act.set_to_position(kp=300.0, kv=10.0)
    act.ctrlrange = [0, C.FINGER_OPEN]
    act.forcerange = [-30, 30]
    act.forcelimited = 1
    ee.add_site(name="tcp", pos=[0, 0, C.TCP_OFFSET], size=[0.006, 0, 0],
                rgba=[1, 0, 0, 1])


def build_model(cube=True, table=True):
    spec = mujoco.MjSpec.from_file(C.ROBOT_XML)
    spec.option.timestep = C.PHYS_DT
    # the HEAL file says inertiafromgeom="false"; AUTO keeps its measured link
    # inertias but lets our new gripper parts get mass/inertia from their boxes
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_AUTO
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # stable with damping
    spec.option.noslip_iterations = 3          # helps the cube not slip in the grip
    spec.visual.global_.offwidth = 1280        # big enough for video frames
    spec.visual.global_.offheight = 960

    # 1) torque motors -> position servos.  The HEAL file drives every joint
    #    with a plain <motor> (torque).  Our planner sends joint ANGLES, so we
    #    delete the motors and add one position actuator per joint (in order).
    for a in list(spec.actuators):
        spec.delete(a)
    for j in C.ARM_JOINTS:
        jt = spec.joint(j)
        jt.armature = C.ARMATURE
        act = spec.add_actuator(name="act_" + j, target=j,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=C.KP_ARM)
        act.ctrlrange = list(jt.range)
    # the force/torque sensors of the original file are not needed
    for s_ in list(spec.sensors):
        spec.delete(s_)

    # 2) the gripper + tool-centre-point site
    add_gripper(spec)

    # 3) make the arm visible (its 'arm' material is fully transparent) and
    #    hide the duplicate collision meshes in group 3.
    for mat in spec.materials:
        if mat.name == "arm":
            mat.rgba = [0.80, 0.80, 0.82, 1]
    for g in spec.geoms:
        if g.name.endswith("_collision"):
            g.group = 3

    # 4) gravity compensation on the robot links (a real controller does this
    #    too).  Without it the position servos sag under the heavy links.
    # 5) robot geoms collide with the world and the cube, NOT with each other
    #    (contype 2 / conaffinity 1): the HEAL convex hulls overlap at the joints.
    robot_bodies = set()
    for b in spec.bodies:
        if b.name.startswith(C.ROBOT_BODY_PREFIX):
            b.gravcomp = 1.0
            robot_bodies.add(b.name)
    for g in spec.geoms:
        if g.parent.name in robot_bodies and g.contype != 0:
            g.contype, g.conaffinity = 2, 1

    # remove the floor/camera/light of the HEAL file (we add our own)
    for g in list(spec.geoms):
        if g.name == "floor":
            spec.delete(g)
    for c in list(spec.cameras):
        if c.name == "scene_3d_view":
            spec.delete(c)
    for l_ in list(spec.lights):
        spec.delete(l_)

    wb = spec.worldbody
    # 2) floor, lights, cameras
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], width=300, height=300)
    spec.add_material(name="grid", textures=["", "grid"], texrepeat=[5, 5],
                      reflectance=0.1)
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05],
                material="grid")
    wb.add_light(pos=[0.5, 0, 2.5], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    wb.add_light(pos=[1.5, 1.0, 1.5], dir=[-1, -0.7, -1], diffuse=[0.3, 0.3, 0.3])
    wb.add_camera(name="front", pos=[1.55, 0.95, 0.95],
                  xyaxes=[-0.52, 0.85, 0, -0.30, -0.18, 0.94])
    wb.add_camera(name="top", pos=[0.5, 0.0, 1.6], xyaxes=[0, -1, 0, 1, 0, 0])

    if table:
        # 3) table: a top slab + 4 legs.  Top surface is exactly at TABLE_H.
        x0, x1 = C.TABLE_X
        y0, y1 = C.TABLE_Y
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
        t = C.TABLE_THICK
        _box(wb, "table_top", (cx, cy, C.TABLE_H - t / 2), (hx, hy, t / 2),
             (0.55, 0.40, 0.28, 1))
        leg_h = (C.TABLE_H - t) / 2
        for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            _box(wb, f"table_leg{i}", (cx + sx * (hx - 0.03), cy + sy * (hy - 0.03), leg_h),
                 (0.02, 0.02, leg_h), (0.35, 0.25, 0.18, 1))
        # 4) tray (static boxes)
        for name, pos, size in tray_geoms():
            _box(wb, name, pos, size, (0.2, 0.55, 0.85, 1))
        # 5) a translucent marker of the cube spawn region (visual only)
        sx0, sx1 = C.SPAWN_X
        sy0, sy1 = C.SPAWN_Y
        _box(wb, "spawn_zone", ((sx0 + sx1) / 2, (sy0 + sy1) / 2, C.TABLE_H + 0.0005),
             ((sx1 - sx0) / 2, (sy1 - sy0) / 2, 0.0005), (0.2, 0.9, 0.3, 0.25),
             contype=0, conaffinity=0)

    if cube:
        # 6) the cube: a body with a free joint (6 DoF) so physics moves it
        body = wb.add_body(name="cube", pos=[0.5, 0.1, C.TABLE_H + C.CUBE_HALF])
        body.add_freejoint(name="cube_free")
        _box(body, "cube_geom", (0, 0, 0), (C.CUBE_HALF,) * 3, (0.9, 0.2, 0.2, 1),
             mass=C.CUBE_MASS, friction=[1.5, 0.05, 0.001], condim=4)
    return spec.compile()


# ---------------------------------------------------------- handy lookups
def robot_collision_geoms(model):
    """IDs of the robot's collision geoms on the wrist, gripper and fingers."""
    names = C.WRIST_BODIES
    ids = []
    for g in range(model.ngeom):
        b = model.body(model.geom_bodyid[g]).name
        if b in names and model.geom_contype[g] != 0:
            ids.append(g)
    return ids


def obstacle_geoms(model):
    """IDs of table + tray geoms (things the robot must not hit)."""
    return [model.geom(n).id for n in
            ["table_top"] + [t[0] for t in tray_geoms()]]


if __name__ == "__main__":
    m = build_model()
    print("bodies:", m.nbody, " nq:", m.nq, " nu:", m.nu)
    print("robot collision geoms:", len(robot_collision_geoms(m)))