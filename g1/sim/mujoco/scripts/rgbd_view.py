import argparse
import os
import sys

# Match .bashrc GL settings to avoid EGL/MuJoCo crashes in this VM.
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
os.environ.setdefault("GLFW_PLATFORM", "x11")
os.environ.setdefault("SDL_VIDEODRIVER", "x11")

import numpy as np

try:
    import cv2
except Exception as exc:
    print("OpenCV (cv2) is required for this script.")
    print(f"Import error: {exc}")
    sys.exit(1)

import mujoco

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import config


def _normalize_depth(depth, max_depth=5.0):
    depth = np.clip(depth, 0.0, max_depth)
    norm = (depth / max_depth * 255.0).astype(np.uint8)
    return cv2.applyColorMap(255 - norm, cv2.COLORMAP_TURBO)


def run_sim():
    model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    data = mujoco.MjData(model)

    width, height = 640, 480
    renderer = mujoco.Renderer(model, width=width, height=height)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    try:
        body_id = model.body("pelvis").id
        cam.lookat[:] = data.xpos[body_id]
    except Exception:
        cam.lookat[:] = np.array([0.0, 0.0, 0.5])
    cam.distance = 2.5
    cam.azimuth = 90.0
    cam.elevation = -15.0

    cv2.namedWindow("RGB (sim)")
    cv2.namedWindow("Depth (sim)")

    while True:
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=cam)
        rgb = renderer.render()
        depth = renderer.render(depth=True)

        rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_vis = _normalize_depth(depth)

        cv2.imshow("RGB (sim)", rgb_bgr)
        cv2.imshow("Depth (sim)", depth_vis)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


def run_robot():
    cv2.namedWindow("RGB (robot)")
    cv2.namedWindow("Depth (robot)")

    empty = np.zeros((480, 640, 3), dtype=np.uint8)

    while True:
        rgb = empty.copy()
        depth = empty.copy()
        cv2.putText(rgb, "No RGB source (SDK)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)
        cv2.putText(depth, "No Depth source (SDK)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)

        cv2.imshow("RGB (robot)", rgb)
        cv2.imshow("Depth (robot)", depth)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Show RGB and depth data.")
    parser.add_argument("--iface", default="lo", help="Network interface (use 'lo' for sim)")
    args = parser.parse_args()

    if args.iface == "lo":
        run_sim()
    else:
        run_robot()


if __name__ == "__main__":
    main()
