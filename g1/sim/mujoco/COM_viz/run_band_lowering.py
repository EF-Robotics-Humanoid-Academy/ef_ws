import argparse
import os
import sys
import threading
import time
import warnings
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

_THIS_DIR = Path(__file__).resolve().parent
_PARENT_DIR = _THIS_DIR.parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))
if str(_PARENT_DIR) not in sys.path:
    sys.path.append(str(_PARENT_DIR))

import config
from unitree_sdk2py_bridge import ElasticBand, UnitreeSdk2Bridge

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
warnings.filterwarnings(
    "ignore", message="pkg_resources is deprecated as an API.*", category=UserWarning
)

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

parser = argparse.ArgumentParser(description="Soft elastic-band lowering for G1")
parser.add_argument("--start-z", type=float, default=3.0, help="Initial band anchor z")
parser.add_argument("--target-z", type=float, default=2.0, help="Final band anchor z")
parser.add_argument("--lower-rate", type=float, default=0.03, help="Anchor lowering speed m/s")
parser.add_argument("--stiffness", type=float, default=60.0, help="Elastic stiffness")
parser.add_argument("--damping", type=float, default=120.0, help="Elastic damping")
parser.add_argument("--max-force", type=float, default=420.0, help="Band force magnitude clamp (N)")
parser.add_argument("--smoothing-tau", type=float, default=0.08, help="Force low-pass time constant")
parser.add_argument("--contact-upscale", type=float, default=0.35, help="Upward force cap while feet in contact, fraction of body weight")
parser.add_argument("--lock-hip-yaw", action="store_true", default=True, help="Lock only hip yaw (z-axis) joints")
parser.add_argument("--no-lock-hip-yaw", action="store_true", help="Do not lock hip yaw joints")
args, _unknown = parser.parse_known_args()

locker = threading.Lock()

model_path = str((_THIS_DIR / config.ROBOT_SCENE).resolve())
mj_model = mujoco.MjModel.from_xml_path(model_path)
mj_data = mujoco.MjData(mj_model)

# Stable contact / joint behavior for touchdown.
mj_model.opt.gravity[:] = np.array([0.0, 0.0, -9.81], dtype=float)
mj_model.opt.iterations = max(mj_model.opt.iterations, 120)
mj_model.opt.ls_iterations = max(mj_model.opt.ls_iterations, 40)
mj_model.dof_damping[:] = np.maximum(mj_model.dof_damping, 0.25)
mj_model.dof_armature[:] = np.maximum(mj_model.dof_armature, 0.03)

mujoco.mj_forward(mj_model, mj_data)

elastic_band = ElasticBand()
elastic_band.stiffness = float(args.stiffness)
elastic_band.damping = float(args.damping)
elastic_band.point = np.array([0.0, 0.0, float(args.start_z)], dtype=float)
elastic_band.length = 0.0

if config.ROBOT in ("h1", "g1"):
    band_attached_link = mj_model.body("torso_link").id
else:
    band_attached_link = mj_model.body("base_link").id

viewer = mujoco.viewer.launch_passive(
    mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
)

viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
mj_model.vis.scale.contactwidth = 0.35
mj_model.vis.scale.contactheight = 0.25
mj_model.vis.scale.forcewidth = 0.06

mj_model.opt.timestep = config.SIMULATE_DT

TOTAL_MASS = float(np.sum(mj_model.body_mass))
BODY_WEIGHT = TOTAL_MASS * 9.81
force_filtered = np.zeros(3, dtype=float)

left_foot_body = mj_model.body("left_ankle_roll_link").id
right_foot_body = mj_model.body("right_ankle_roll_link").id
pelvis_body = mj_model.body("pelvis").id
torso_body = mj_model.body("torso_link").id
foot_geom_ids = {
    gid
    for gid in range(mj_model.ngeom)
    if int(mj_model.geom_bodyid[gid]) in (left_foot_body, right_foot_body)
}
try:
    floor_geom_id = mj_model.geom("floor").id
except Exception:
    floor_geom_id = -1

lock_hip_yaw = bool(args.lock_hip_yaw) and (not bool(args.no_lock_hip_yaw))
hip_yaw_qpos = np.zeros(0, dtype=np.int32)
hip_yaw_dof = np.zeros(0, dtype=np.int32)
hip_yaw_act = np.zeros(0, dtype=np.int32)
hip_yaw_target = np.zeros(0, dtype=float)
if lock_hip_yaw:
    hip_ids = []
    for name in ("left_hip_yaw_joint", "right_hip_yaw_joint"):
        jid = mj_model.joint(name).id
        hip_ids.append((int(mj_model.jnt_qposadr[jid]), int(mj_model.jnt_dofadr[jid])))
    hip_yaw_qpos = np.asarray([q for q, _ in hip_ids], dtype=np.int32)
    hip_yaw_dof = np.asarray([d for _, d in hip_ids], dtype=np.int32)
    act_ids = []
    for name in ("left_hip_yaw_joint", "right_hip_yaw_joint"):
        try:
            act_ids.append(int(mj_model.actuator(name).id))
        except Exception:
            pass
    hip_yaw_act = np.asarray(act_ids, dtype=np.int32)
    hip_yaw_target = mj_data.qpos[hip_yaw_qpos].copy()


def _feet_in_contact(data: mujoco.MjData) -> bool:
    for i in range(data.ncon):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        if floor_geom_id >= 0:
            if (g1 in foot_geom_ids and g2 == floor_geom_id) or (
                g2 in foot_geom_ids and g1 == floor_geom_id
            ):
                return True
        else:
            if g1 in foot_geom_ids or g2 in foot_geom_ids:
                return True
    return False


def _cap_force(f: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(f))
    if n <= max_norm or n < 1e-9:
        return f
    return f * (max_norm / n)


def SimulationThread():
    global force_filtered

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    target_z = float(args.target_z)
    lower_rate = max(1e-4, float(args.lower_rate))

    if not viewer.is_running():
        return

    status_t = 0.0
    while viewer.is_running():
        step_start = time.perf_counter()

        with locker:
            if lock_hip_yaw and hip_yaw_qpos.size:
                mj_data.qpos[hip_yaw_qpos] = hip_yaw_target
                mj_data.qvel[hip_yaw_dof] = 0.0

            unitree.ApplySportCommand()
            unitree.ApplyHandCommand()

            # Smoothly lower anchor toward target z.
            z_now = float(elastic_band.point[2])
            z_next = max(target_z, z_now - lower_rate * mj_model.opt.timestep)
            elastic_band.point[2] = z_next

            if elastic_band.enable:
                f_raw = elastic_band.Advance(mj_data.qpos[:3], mj_data.qvel[:3])
                feet_contact = _feet_in_contact(mj_data)

                if feet_contact:
                    f_raw[0] *= 0.2
                    f_raw[1] *= 0.2
                    f_raw[2] = min(f_raw[2], float(args.contact_upscale) * BODY_WEIGHT)

                f_raw = _cap_force(f_raw, float(args.max_force))

                alpha = mj_model.opt.timestep / (
                    max(1e-4, float(args.smoothing_tau)) + mj_model.opt.timestep
                )
                force_filtered = (1.0 - alpha) * force_filtered + alpha * f_raw
                mj_data.xfrc_applied[band_attached_link, :3] = force_filtered

            if lock_hip_yaw and hip_yaw_act.size:
                mj_data.ctrl[hip_yaw_act] = 0.0

            mujoco.mj_step(mj_model, mj_data)

            if lock_hip_yaw and hip_yaw_qpos.size:
                mj_data.qpos[hip_yaw_qpos] = hip_yaw_target
                mj_data.qvel[hip_yaw_dof] = 0.0

        status_t += mj_model.opt.timestep
        if status_t > 1.0:
            status_t = 0.0
            pelvis_z = float(mj_data.xpos[pelvis_body, 2])
            torso_z = float(mj_data.xpos[torso_body, 2])
            print(
                f"[band] z={elastic_band.point[2]:.3f} contact={_feet_in_contact(mj_data)} "
                f"Fz={force_filtered[2]:.1f}N pelvis_z={pelvis_z:.3f} torso_z={torso_z:.3f}",
                flush=True,
            )

        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    if not viewer.is_running():
        return

    while viewer.is_running():
        with locker:
            viewer.sync()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = threading.Thread(target=PhysicsViewerThread)
    sim_thread = threading.Thread(target=SimulationThread)
    viewer_thread.start()
    sim_thread.start()
