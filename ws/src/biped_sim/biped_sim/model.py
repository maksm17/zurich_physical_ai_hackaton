"""Build a MuJoCo model of the biped straight from biped.xacro.

Adds: free-floating base, floor, ST3215-like position actuators, IMU sensors.
"""
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

JOINTS = ['l_up', 'l_mid', 'l_down', 'r_up', 'r_mid', 'r_down']

SERVO = dict(kp=8.0, kv=0.15, max_torque=2.9, armature=0.005, damping=0.05, frictionloss=0.01)


def urdf_from_xacro(xacro_path):
    """Accepts .xacro (needs ROS xacro) or an already expanded .urdf (e.g. for the RL venv)."""
    if xacro_path.endswith('.urdf'):
        root = ET.parse(xacro_path).getroot()
    else:
        import xacro
        root = ET.fromstring(xacro.process_file(xacro_path, mappings={'use_mock': 'true'}).toxml())
    for tag in ('ros2_control', 'gazebo'):
        for el in root.findall(tag):
            root.remove(el)
    for mesh in root.iter('mesh'):
        mesh.set('filename', re.sub(r'^(file|package)://', '', mesh.get('filename')))
    mj = ET.SubElement(root, 'mujoco')
    ET.SubElement(mj, 'compiler', discardvisual='false', balanceinertia='true',
                  fusestatic='false')
    return ET.tostring(root, encoding='unicode')


def build_spec(urdf, servo=SERVO):
    spec = mujoco.MjSpec.from_string(urdf)
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    world = spec.worldbody
    world.add_light(pos=[0, 0, 2], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    tex = spec.add_texture(name='grid', type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], width=300, height=300)
    mat = spec.add_material(name='grid', texrepeat=[20, 20], reflectance=0.1)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1],
                   material='grid', friction=[1.0, 0.005, 0.0001],
                   conaffinity=3)

    base = spec.body('base_link')
    base.add_freejoint(name='root')

    imu = spec.body('imu_link')
    imu.add_site(name='imu')
    spec.add_sensor(name='imu_quat', type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    spec.add_sensor(name='imu_gyro', type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    spec.add_sensor(name='imu_acc', type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')

    for name in JOINTS:
        j = spec.joint(name)
        j.armature = servo['armature']
        j.damping[0] = servo['damping']
        j.frictionloss = servo['frictionloss']
        act = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=servo['kp'], kv=servo['kv'])
        act.forcelimited = True
        act.forcerange = [-servo['max_torque'], servo['max_torque']]
        act.ctrllimited = True
        act.ctrlrange = j.range
    return spec


def lowest_point(model, data):
    """Lowest z of all colliding robot geoms (meshes and boxes) in the current pose."""
    zmin = np.inf
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0 or model.geom_contype[g] == 0:
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            m = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            r, h = model.geom_size[g][:2]
            ang = np.linspace(0, 2 * np.pi, 72)
            v = np.array([[r * np.cos(a), r * np.sin(a), s * h] for a in ang for s in (-1, 1)])
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
            zmin = min(zmin, data.geom_xpos[g][2] - model.geom_size[g][0])
            continue
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            v = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            v = v * model.geom_size[g]
        else:
            continue
        z = data.geom_xpos[g][2] + v @ data.geom_xmat[g].reshape(3, 3)[2]
        zmin = min(zmin, z.min())
    return zmin


def add_soles(spec, model, data, size):
    """Replace the (rounded) foot mesh contact by a flat box sole of size [x, y, thickness].

    The sole is centred under the foot mesh footprint; its bottom is at the mesh's lowest point.
    """
    for foot in ('l_foot_1', 'r_foot_1'):
        bid = model.body(foot).id
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != bid or model.geom_contype[g] == 0:
                continue
            m = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
            w = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
            spec.geom(model.geom(g).name).contype = 0
            spec.geom(model.geom(g).name).conaffinity = 0
        center = np.array([(w[:, 0].min() + w[:, 0].max()) / 2,
                           (w[:, 1].min() + w[:, 1].max()) / 2,
                           w[:, 2].min() + size[-1] / 2])
        if len(size) == 2:
            center[2] = w[:, 2].min() + size[0]
            local = data.xmat[bid].reshape(3, 3).T @ (center - data.xpos[bid])
            spec.body(foot).add_geom(name=foot + '_sole', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                                     size=[size[0], size[1] / 2, 0], pos=local,
                                     quat=[np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0],
                                     rgba=[0.9, 0.5, 0.1, 0.15], friction=[1.0, 0.005, 0.0001],
                                     contype=2, conaffinity=0)
            continue
        if len(size) == 1:
            center[2] = w[:, 2].min() + size[0]
            local = data.xmat[bid].reshape(3, 3).T @ (center - data.xpos[bid])
            spec.body(foot).add_geom(name=foot + '_sole', type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                     size=[size[0], 0, 0], pos=local, rgba=[0.9, 0.5, 0.1, 0.15],
                                     friction=[1.0, 0.005, 0.0001], contype=2, conaffinity=0)
            continue
        local = data.xmat[bid].reshape(3, 3).T @ (center - data.xpos[bid])
        spec.body(foot).add_geom(name=foot + '_sole', type=mujoco.mjtGeom.mjGEOM_BOX,
                                 size=[size[0] / 2, size[1] / 2, size[2] / 2], pos=local,
                                 rgba=[0.9, 0.5, 0.1, 1], friction=[1.0, 0.005, 0.0001],
                                 contype=2, conaffinity=0)


def cut_feet(spec, model, data, depth):
    """Flatten the foot bottoms like filing them down: cut the collision mesh `depth` [m] above
    its lowest point. Returns the flat patch size per foot (x, y) in metres."""
    patches = {}
    for foot in ('l_foot_1', 'r_foot_1'):
        bid = model.body(foot).id
        for g in range(model.ngeom):
            if (model.geom_bodyid[g] != bid or model.geom_contype[g] == 0
                    or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH):
                continue
            m = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
            w = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
            plane = w[:, 2].min() + depth
            low = w[:, 2] <= plane
            patches[foot] = (np.ptp(w[low, 0]), np.ptp(w[low, 1]))
            w[low, 2] = plane
            local = (w - data.xpos[bid]) @ data.xmat[bid].reshape(3, 3)
            spec.geom(model.geom(g).name).contype = 0
            spec.geom(model.geom(g).name).conaffinity = 0
            spec.add_mesh(name=foot + '_cut', uservert=local.flatten().tolist())
            spec.body(foot).add_geom(name=foot + '_cut', type=mujoco.mjtGeom.mjGEOM_MESH,
                                     meshname=foot + '_cut', rgba=[0.9, 0.5, 0.1, 1],
                                     friction=[1.0, 0.005, 0.0001], contype=2, conaffinity=0)
    return patches


def build_model(xacro_path, servo=SERVO, sole=None, cut=0.0):
    """Returns (model, spec). Robot is placed with its feet just touching the floor.

    cut: flatten the CAD feet this much [m] (filed-down soles); used when sole is None.
    sole: None = use the foot meshes as they are, [x, y, thickness] for flat box soles,
          [R] for a spherical rocker sole, or [R, width] for a cylindrical rocker
          (curved front-back only).
    """
    spec = build_spec(urdf_from_xacro(xacro_path), servo)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if sole is not None or cut > 0:
        for g in spec.geoms:
            if not g.name:
                g.name = f'geom_{id(g)}'
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        if sole is not None:
            add_soles(spec, model, data, sole)
        else:
            cut_feet(spec, model, data, cut)
        model = spec.compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
    spec.body('base_link').pos[2] += -lowest_point(model, data) + 0.001
    return spec.compile(), spec
