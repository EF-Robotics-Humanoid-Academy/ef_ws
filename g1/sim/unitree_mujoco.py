import time
import os
import warnings
import argparse
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core import channel as channel_module
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
warnings.filterwarnings(
    "ignore", message="pkg_resources is deprecated as an API.*", category=UserWarning
)

from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

parser = argparse.ArgumentParser(description="Unitree MuJoCo simulator")
parser.add_argument(
    "--hanged",
    action="store_true",
    help="Enable elastic band (hanged mode).",
)
args, _unknown = parser.parse_known_args()

# Override config to avoid editing config.py for hanged mode.
config.ENABLE_ELASTIC_BAND = bool(args.hanged)

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

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

def _disable_opengl_effects(viewer_handle):
    # Turn off costly visual effects by default.
    try:
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_SHADOW] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_REFLECTION] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_HAZE] = False
        viewer_handle.opt.flags[mujoco.mjtVisFlag.mjVIS_SKYBOX] = False
    except Exception:
        # If flags are unavailable, fail silently to avoid breaking the viewer.
        pass

_disable_opengl_effects(viewer)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        try:
            unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
        except SystemExit:
            print("Joystick init failed; continuing without gamepad.", flush=True)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    if not viewer.is_running():
        return

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        unitree.ApplySportCommand()
        unitree.ApplyHandCommand()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    if not viewer.is_running():
        return

    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
