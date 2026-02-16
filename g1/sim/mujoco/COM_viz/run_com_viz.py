import os
import sys
import time
import threading
import argparse
import warnings
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core import channel as channel_module

_THIS_DIR = Path(__file__).resolve().parent
_PARENT_DIR = _THIS_DIR.parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))
import config
if str(_PARENT_DIR) not in sys.path:
    sys.path.append(str(_PARENT_DIR))

# Keep launcher output clean from pygame/pkg_resources deprecation noise.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
warnings.filterwarnings(
    "ignore", message="pkg_resources is deprecated as an API.*", category=UserWarning
)

from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

locker = threading.Lock()

channel_module.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""

if getattr(config, "CYCLONEDDS_URI", None):
    os.environ["CYCLONEDDS_URI"] = config.CYCLONEDDS_URI

parser = argparse.ArgumentParser(description="G1 COM/weight-force visualizer")
parser.add_argument(
    "--hanged",
    action="store_true",
    help="Enable elastic band regardless of config.",
)
parser.add_argument(
    "--no-hanged",
    action="store_true",
    help="Disable elastic band regardless of config.",
)
args, _unknown = parser.parse_known_args()

enable_elastic_band = bool(getattr(config, "ENABLE_ELASTIC_BAND", False))
if args.hanged:
    enable_elastic_band = True
if args.no_hanged:
    enable_elastic_band = False
lock_hip_yaw = bool(getattr(config, "LOCK_HIP_YAW", False))

model_path = str((_THIS_DIR / config.ROBOT_SCENE).resolve())
mj_model = mujoco.MjModel.from_xml_path(model_path)
mj_data = mujoco.MjData(mj_model)

# Force gravity on.
mj_model.opt.gravity[:] = np.array([0.0, 0.0, -9.81], dtype=float)

mujoco.mj_forward(mj_model, mj_data)

elastic_band = None
band_attached_link = -1
if enable_elastic_band:
    elastic_band = ElasticBand()
    # Lower anchor and soften support so knees settle with slight bend.
    elastic_band.point = np.array([0.0, 0.0, 3.0], dtype=float)
    elastic_band.length = 0.025
    if config.ROBOT in ("h1", "g1"):
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)


def _disable_opengl_effects(viewer_handle):
    try:
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_SHADOW] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_REFLECTION] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_HAZE] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_SKYBOX] = False
    except Exception:
        pass


_disable_opengl_effects(viewer)
# Make contact diagnostics visible by default.
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

mj_model.opt.timestep = config.SIMULATE_DT
TOTAL_MASS = float(np.sum(mj_model.body_mass))
FORCE_TO_LENGTH = 0.0012  # meters per Newton for visualization
# Increase visibility of contact points/force vectors.
mj_model.vis.scale.contactwidth = 0.35
mj_model.vis.scale.contactheight = 0.25
mj_model.vis.scale.forcewidth = 0.06

HIP_YAW_JOINT_NAMES = (
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
)
hip_lock_qpos = []
hip_lock_dof = []
hip_lock_act = []
if lock_hip_yaw:
    for name in HIP_YAW_JOINT_NAMES:
        jid = mj_model.joint(name).id
        hip_lock_qpos.append(int(mj_model.jnt_qposadr[jid]))
        hip_lock_dof.append(int(mj_model.jnt_dofadr[jid]))
        try:
            hip_lock_act.append(int(mj_model.actuator(name).id))
        except Exception:
            pass
    hip_lock_qpos = np.asarray(hip_lock_qpos, dtype=np.int32)
    hip_lock_dof = np.asarray(hip_lock_dof, dtype=np.int32)
    hip_lock_act = np.asarray(hip_lock_act, dtype=np.int32)
    hip_lock_target = mj_data.qpos[hip_lock_qpos].copy()
else:
    hip_lock_target = np.zeros(0, dtype=float)


def _update_com_vector(viewer_handle, mj_data_handle):
    com = np.array(mj_data_handle.subtree_com[0], dtype=float)
    gravity = np.array(mj_model.opt.gravity, dtype=float)
    # Weight force in Newtons (downward when gravity is negative z).
    weight_force = TOTAL_MASS * gravity
    start = com
    end = com + FORCE_TO_LENGTH * weight_force

    geom = viewer_handle.user_scn.geoms[0]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        np.array([0.012, 0.012, 0.012], dtype=float),
        np.zeros(3, dtype=float),
        np.eye(3).reshape(-1),
        np.array([1.0, 0.2, 0.2, 1.0], dtype=float),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        0.008,
        start.astype(np.float64),
        end.astype(np.float64),
    )
    viewer_handle.user_scn.ngeom = 1


def SimulationThread():
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    if not viewer.is_running():
        return

    while viewer.is_running():
        step_start = time.perf_counter()

        with locker:
            if lock_hip_yaw and hip_lock_qpos.size:
                mj_data.qpos[hip_lock_qpos] = hip_lock_target
                mj_data.qvel[hip_lock_dof] = 0.0

            unitree.ApplySportCommand()
            unitree.ApplyHandCommand()
            if elastic_band is not None and elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
            if lock_hip_yaw and hip_lock_act.size:
                mj_data.ctrl[hip_lock_act] = 0.0
            mujoco.mj_step(mj_model, mj_data)

            if lock_hip_yaw and hip_lock_qpos.size:
                mj_data.qpos[hip_lock_qpos] = hip_lock_target
                mj_data.qvel[hip_lock_dof] = 0.0

        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    if not viewer.is_running():
        return

    while viewer.is_running():
        with locker:
            _update_com_vector(viewer, mj_data)
            viewer.sync()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = threading.Thread(target=PhysicsViewerThread)
    sim_thread = threading.Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
