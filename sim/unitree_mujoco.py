import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading
import os

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()
_ctrl_print_every = 50
_ctrl_print_count = 0

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

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model
    global _ctrl_print_count
    print("SimulationThread started", flush=True)
    print(f"SimulationThread viewer.is_running={viewer.is_running()}", flush=True)

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
        print("SimulationThread exiting: viewer not running", flush=True)
        return

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        unitree.ApplySportCommand()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        if _ctrl_print_every > 0:
            _ctrl_print_count += 1
            if _ctrl_print_count % _ctrl_print_every == 0:
                max_abs = float(max(abs(mj_data.ctrl)))
                print(
                    f"ctrl[0:6]={mj_data.ctrl[:6]} max|ctrl|={max_abs:.3f} qpos[7:13]={mj_data.qpos[7:13]}",
                    flush=True,
                )

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    print("PhysicsViewerThread started", flush=True)
    if not viewer.is_running():
        print("PhysicsViewerThread exiting: viewer not running", flush=True)
        return

    tick = 0
    while viewer.is_running():
        tick += 1
        if tick % 200 == 0:
            print("PhysicsViewerThread tick", flush=True)
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    print("Starting unitree_mujoco.py", flush=True)
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
    print("Threads launched", flush=True)
