"""run_geoff_stack.py – one-stop script that starts

1.  Unitree G-1 tele-operation (keyboard_controller.py)
2.  RealSense UDP viewer (receive_realsense_gst.py – RGB & colourised depth)
3.  Live LiDAR SLAM for the Livox MID-360 (live_slam.py)

and combines the *visual* output into **one single OpenCV window** that looks
roughly like::

    ┌──────────────────────────── RGB (640×480) ───────────────────────────┐
    │                                                                     │
    ├────────────────────────── Depth (640×480) ───────────────────────────┤
    │                                                                     │
    └─────────────────────── 2-D SLAM preview (480×480) ───────────────────┘

The script has been written for convenience rather than scientific rigor –
it glues the three existing modules together with a minimum of changes and
contains a fair bit of *best-effort* fall-back logic so it can still be
imported on machines that lack some of the heavyweight runtime dependencies
(GStreamer, Livox SDK, Open3D, Unitree SDK, …).  If a component is missing a
clear warning is printed and the corresponding pane simply shows a solid
grey background.

Usage
-----

    python run_geoff_stack.py [--iface IFACE]

``--iface`` specifies the network interface that is connected to the Unitree
G-1 and is forwarded to ``hanger_boot_sequence()``.  All other parameters are
identical to the individual helper scripts and can still be tweaked via the
respective environment variables (e.g. ``LIVOX_PRESET``).

Implementation notes
--------------------

* **Threading** – the three subsystems run in background threads and publish
  their most recent numpy images in a shared dictionary (protected by a very
  light-weight ``threading.Lock`` since we only replace whole numpy arrays).
* **SLAM visualisation** – in order to stick to *one* GUI window we replace
  ``live_slam._Viewer`` with a minimal off-screen variant that renders a
  simple bird-eye 2-D projection of the local map into a 480×480 numpy
  canvas.  This keeps the dependency surface small (no need for an OpenGL
  context) and plays well with OpenCV.
* **Keyboard tele-op** – copied & trimmed from ``keyboard_controller.py``.  We
  keep the exact key-bindings and update the current target velocities in
  ``shared_state`` so they can be overlayed on the composite canvas.

If you need richer 3-D interaction in the future consider migrating the
front-end to ``open3d.visualization.gui`` or PyQt – both can embed the full
Open3D widget next to normal images – but that would add quite some
additional boiler-plate.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any, Dict, Optional


def _import_cv2():  # pragma: no cover - small helper
    # Prefer system OpenCV (often GTK-based) to avoid Qt plugin crashes.
    for p in ("/usr/lib/python3/dist-packages", "/usr/lib/python3.12/dist-packages"):
        if p not in sys.path:
            sys.path.insert(0, p)
    import cv2  # type: ignore

    return cv2

# ---------------------------------------------------------------------------
# Shared state between threads (very small – only last frame / numbers).
# ---------------------------------------------------------------------------

_state_lock = threading.Lock()
_state: Dict[str, Any] = {
    "rgbd": None,        # numpy BGR image from RealSense (1280×480)
    "slam": None,        # numpy BGR image (480×480)
    "vel": (0.0, 0.0, 0.0),  # current (vx, vy, omega)
}


# ---------------------------------------------------------------------------
# 1.  RealSense receiver  (GStreamer → numpy) – adapted from
#     receive_realsense_gst.py but trimmed to only publish the combined image.
# ---------------------------------------------------------------------------


def _rx_realsense(stop: threading.Event) -> None:  # pragma: no cover – HW req.
    try:
        try:
            import gi  # type: ignore
        except ModuleNotFoundError:
            # Many venvs omit system dist-packages; try the system GI path.
            for p in (
                "/usr/lib/python3/dist-packages",
                "/usr/lib/python3.12/dist-packages",
            ):
                if p not in sys.path:
                    sys.path.append(p)
            import gi  # type: ignore

        gi.require_version("Gst", "1.0")
        gi.require_version("GstApp", "1.0")
        from gi.repository import Gst, GstApp  # type: ignore

        import numpy as np  # pylint: disable=import-error
        cv2 = _import_cv2()

        RGB_PORT, DEPTH_PORT, WIDTH, HEIGHT, FPS = 5600, 5602, 640, 480, 30

        def _build_sink(port: int, payload: int) -> tuple[Any, Any]:
            pipeline = Gst.parse_launch(
                f"udpsrc port={port} caps=application/x-rtp,media=video,encoding-name=H264,payload={payload} ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=true sync=false drop=true"
            )
            sink = pipeline.get_by_name("sink")
            return sink, pipeline

        Gst.init(None)

        rgb_sink, rgb_pipe = _build_sink(RGB_PORT, 96)
        d_sink, d_pipe = _build_sink(DEPTH_PORT, 97)

        for p in (rgb_pipe, d_pipe):
            p.set_state(Gst.State.PLAYING)

        last = time.perf_counter()

        while not stop.is_set():
            sample_rgb = rgb_sink.emit("try-pull-sample", Gst.SECOND // FPS)
            sample_d = d_sink.emit("try-pull-sample", Gst.SECOND // FPS)

            if not sample_rgb or not sample_d:
                time.sleep(0.005)
                continue

            buf_rgb = sample_rgb.get_buffer()
            buf_d = sample_d.get_buffer()

            rgb = np.frombuffer(buf_rgb.extract_dup(0, buf_rgb.get_size()), dtype=np.uint8)
            rgb = rgb.reshape((HEIGHT, WIDTH, 3))

            depth_bgr = np.frombuffer(buf_d.extract_dup(0, buf_d.get_size()), dtype=np.uint8)
            depth_bgr = depth_bgr.reshape((HEIGHT, WIDTH, 3))

            combo = cv2.hconcat([rgb, depth_bgr])

            fps = 1.0 / (time.perf_counter() - last)
            last = time.perf_counter()
            cv2.putText(combo, f"RGB+Depth  {fps:5.1f} FPS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            with _state_lock:
                _state["rgbd"] = combo

        # Tear-down ------------------------------------------------------
        for p in (rgb_pipe, d_pipe):
            p.set_state(Gst.State.NULL)

    except Exception as exc:  # pylint: disable=broad-except
        print("[run_geoff_stack] RealSense receiver disabled:", exc, file=sys.stderr)


# ---------------------------------------------------------------------------
# 2.  Live SLAM  →  2-D bird-eye preview (numpy 480×480)
# ---------------------------------------------------------------------------


def _monkey_patch_slam_viewer() -> None:  # pragma: no cover – small helper
    """Replace live_slam._Viewer with a minimal off-screen variant.

    The new implementation only keeps the public contract (``push()`` &
    ``tick()``) but instead of opening an OpenGL window it renders a 2-D top-
    down scatter plot into a small numpy canvas so we can show it next to the
    camera streams inside the OpenCV mosaic.
    """

    try:
        import numpy as np  # type: ignore
        cv2 = _import_cv2()

        import live_slam as _ls  # type: ignore

        class _MiniViewer:  # pylint: disable=too-few-public-methods
            def __init__(self) -> None:
                self._latest_pts: Optional[np.ndarray] = None
                self._img: Optional[np.ndarray] = None

            # ----------------------------------------------
            def push(self, xyz: np.ndarray, _pose: np.ndarray):
                # Save copy – callback comes from background thread
                self._latest_pts = xyz

            # ----------------------------------------------
            def tick(self) -> bool:  # noqa: D401  – same signature as original
                if self._latest_pts is None:
                    return True  # alive

                pts = self._latest_pts
                self._latest_pts = None

                if pts.shape[0] == 0:
                    return True

                # ----------------  very small & very fast scatter -> canvas
                x, y = pts[:, 0], pts[:, 1]
                min_x, max_x = float(x.min()), float(x.max())
                min_y, max_y = float(y.min()), float(y.max())

                span = max(max_x - min_x, max_y - min_y, 1e-6)
                scale = 470.0 / span  # leave small margin

                img = np.zeros((480, 480, 3), dtype=np.uint8)

                # Map x/y → pixel
                px = ((x - min_x) * scale + 5).astype(np.int32)
                py = ((y - min_y) * scale + 5).astype(np.int32)
                py = 479 - py  # flip so +y is up in the image

                img[py.clip(0, 479), px.clip(0, 479)] = (0, 255, 0)

                # Simple bounding box
                cv2.rectangle(img, (0, 0), (479, 479), (255, 255, 255), 1)

                self._img = img

                with _state_lock:
                    _state["slam"] = self._img

                return True  # keep running

            # ----------------------------------------------
            def close(self):  # kept for compatibility with LiveSLAMDemo.shutdown
                pass

        # Monkey-patch 👍
        _ls._Viewer = _MiniViewer  # type: ignore[attr-defined]

    except Exception as exc:  # pylint: disable=broad-except
        print("[run_geoff_stack] SLAM viewer patch failed:", exc, file=sys.stderr)


def _run_slam_dds(stop: threading.Event) -> None:  # pragma: no cover – HW req.
    """DDS-based SLAM preview using live pointclouds (no Open3D)."""
    try:
        import numpy as np  # type: ignore
        cv2 = _import_cv2()
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize  # type: ignore
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # type: ignore

        iface = os.environ.get("G1_SLAM_IFACE", "")
        domain = int(os.environ.get("G1_SLAM_DOMAIN", "0"))
        topic = os.environ.get("G1_SLAM_TOPIC", "rt/utlidar/cloud_livox_mid360")
        zmin = float(os.environ.get("G1_SLAM_ZMIN", "-0.5"))
        zmax = float(os.environ.get("G1_SLAM_ZMAX", "1.5"))
        stride = int(os.environ.get("G1_SLAM_STRIDE", "4"))
        span = float(os.environ.get("G1_SLAM_RANGE", "12.0"))

        if iface:
            ChannelFactoryInitialize(domain, iface)
        else:
            ChannelFactoryInitialize(domain)

        latest: dict[str, Any] = {"msg": None}

        def _cb(msg: Any) -> None:
            latest["msg"] = msg

        sub = ChannelSubscriber(topic, PointCloud2_)
        sub.Init(_cb, 10)

        while not stop.is_set():
            msg = latest["msg"]
            if msg is None:
                time.sleep(0.02)
                continue

            try:
                fields = {f.name: f for f in msg.fields}
                if "x" not in fields or "y" not in fields:
                    time.sleep(0.02)
                    continue
                point_step = int(msg.point_step)
                if point_step <= 0:
                    time.sleep(0.02)
                    continue
                data = bytes(msg.data)
                if not data:
                    time.sleep(0.02)
                    continue
                xoff = int(fields["x"].offset)
                yoff = int(fields["y"].offset)
                zoff = int(fields["z"].offset) if "z" in fields else xoff + 8

                dtype = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": ["<f4", "<f4", "<f4"],
                        "offsets": [xoff, yoff, zoff],
                        "itemsize": point_step,
                    }
                )
                arr = np.frombuffer(data, dtype=dtype, count=len(data) // point_step)
                xs = arr["x"][::stride]
                ys = arr["y"][::stride]
                zs = arr["z"][::stride]

                img = np.zeros((480, 480, 3), dtype=np.uint8)
                center = 240
                scale = 480.0 / (2.0 * span)
                for x, y, z in zip(xs, ys, zs):
                    if z < zmin or z > zmax:
                        continue
                    if abs(x) > span or abs(y) > span:
                        continue
                    px = int(center + x * scale)
                    py = int(center - y * scale)
                    if 0 <= px < 480 and 0 <= py < 480:
                        img[py, px] = (0, 200, 255)

                cv2.rectangle(img, (0, 0), (479, 479), (255, 255, 255), 1)

                with _state_lock:
                    _state["slam"] = img
            except Exception:
                pass

            time.sleep(0.02)

    except Exception as exc:  # pylint: disable=broad-except
        print("[run_geoff_stack] DDS SLAM disabled:", exc, file=sys.stderr)


def _run_slam(stop: threading.Event) -> None:  # pragma: no cover – HW req.
    """Default to DDS SLAM preview to avoid Open3D crashes."""
    use_open3d = os.environ.get("G1_SLAM_OPEN3D", "0") == "1"
    if not use_open3d:
        _run_slam_dds(stop)
        return

    try:
        # Guard: Open3D CUDA builds can hard-crash the process on import if
        # the system driver is too old. Probe in a subprocess first.
        import subprocess

        probe = subprocess.run(
            [sys.executable, "-c", "import open3d"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if probe.returncode != 0:
            raise RuntimeError("Open3D import failed (likely CUDA driver/runtime mismatch)")

        _monkey_patch_slam_viewer()

        import live_slam as _ls  # type: ignore  # now uses patched viewer

        demo = _ls.LiveSLAMDemo()  # type: ignore[attr-defined]

        while not stop.is_set():
            # Tick just to let the patched viewer process latest cloud
            if not demo._viewer.tick():  # type: ignore[attr-defined]
                break
            time.sleep(0.01)

        demo.shutdown()

    except Exception as exc:  # pylint: disable=broad-except
        print("[run_geoff_stack] SLAM disabled:", exc, file=sys.stderr)


# ---------------------------------------------------------------------------
# 3.  Keyboard tele-operation (pynput → Unitree G-1)
# ---------------------------------------------------------------------------


def _keyboard_thread(stop: threading.Event, iface: str):
    try:
        from hanger_boot_sequence import hanger_boot_sequence  # type: ignore
        from pynput.keyboard import Listener, Key, KeyCode  # type: ignore

        bot = hanger_boot_sequence(iface=iface)

        vx = vy = omega = 0.0
        LIN_STEP, ANG_STEP = 0.05, 0.2
        SEND_PERIOD = 0.1

        def _clamp(value: float, limit: float = 0.6) -> float:
            return max(-limit, min(limit, value))

        pressed: set[Any] = set()

        def on_press(k):  # noqa: D401 – callback
            if isinstance(k, KeyCode) and k.char is not None:
                pressed.add(k.char.lower())
            else:
                pressed.add(k)

        def on_release(k):
            if isinstance(k, KeyCode) and k.char is not None:
                pressed.discard(k.char.lower())
            else:
                pressed.discard(k)

        listener = Listener(on_press=on_press, on_release=on_release)
        listener.start()

        last_send = 0.0

        def _is(name: str) -> bool:
            if name == "space":
                return Key.space in pressed
            if name == "esc":
                return Key.esc in pressed
            return name in pressed

        while not stop.is_set():
            if _is("w") and not _is("s"):
                vx = _clamp(vx + LIN_STEP)
            elif _is("s") and not _is("w"):
                vx = _clamp(vx - LIN_STEP)
            else:
                vx = 0.0

            if _is("q") and not _is("e"):
                vy = _clamp(vy + LIN_STEP)
            elif _is("e") and not _is("q"):
                vy = _clamp(vy - LIN_STEP)
            else:
                vy = 0.0

            if _is("a") and not _is("d"):
                omega = _clamp(omega + ANG_STEP)
            elif _is("d") and not _is("a"):
                omega = _clamp(omega - ANG_STEP)
            else:
                omega = 0.0

            if _is("space"):
                vx = vy = omega = 0.0

            # Exit keys --------------------------------------------------
            if _is("z"):
                bot.Damp()
                break
            if _is("esc"):
                bot.StopMove(); bot.ZeroTorque(); break

            now = time.time()
            if now - last_send >= SEND_PERIOD:
                bot.Move(vx, vy, omega, continous_move=True)
                last_send = now

                with _state_lock:
                    _state["vel"] = (vx, vy, omega)

            time.sleep(0.005)

    except Exception as exc:  # pylint: disable=broad-except
        print("[run_geoff_stack] Keyboard / G-1 control disabled:", exc, file=sys.stderr)


# ---------------------------------------------------------------------------
# Top-level composite GUI – OpenCV only
# ---------------------------------------------------------------------------


def _compose_canvas() -> "Optional['np.ndarray']":  # type: ignore[name-defined]
    import numpy as np  # local import to avoid hard dep if script is only imported
    cv2 = _import_cv2()

    with _state_lock:
        rgbd = _state.get("rgbd")
        slam = _state.get("slam")
        vx, vy, om = _state.get("vel", (0.0, 0.0, 0.0))

    # Place-holders -------------------------------------------------------
    if rgbd is None:
        rgbd = np.full((480, 1280, 3), 80, dtype=np.uint8)
        cv2.putText(rgbd, "No RealSense stream", (380, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

    if slam is None:
        slam = np.full((480, 480, 3), 60, dtype=np.uint8)
        cv2.putText(slam, "No SLAM data", (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

    # Compose – simple vertical stack
    top = rgbd
    bottom = cv2.copyMakeBorder(slam, 0, 0, 0, max(0, top.shape[1] - slam.shape[1]), cv2.BORDER_CONSTANT, value=(0, 0, 0))

    canvas = np.vstack([top, bottom])

    # HUD with current velocities
    txt = f"vx {vx:+.2f}  vy {vy:+.2f}  omega {om:+.2f}   –  Z: quit  ESC: e-stop"
    cv2.rectangle(canvas, (0, canvas.shape[0] - 40), (canvas.shape[1], canvas.shape[0]), (0, 0, 0), -1)
    cv2.putText(canvas, txt, (10, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    return canvas


# ---------------------------------------------------------------------------


def main() -> None:  # noqa: D401
    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp68s0f1", help="network interface connected to Unitree G-1")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--slam-topic", default="rt/utlidar/cloud_livox_mid360", help="DDS pointcloud topic for SLAM view")
    parser.add_argument("--slam-open3d", action="store_true", help="Use Open3D live_slam instead of DDS viewer")
    args = parser.parse_args()

    os.environ["G1_SLAM_IFACE"] = args.iface
    os.environ["G1_SLAM_DOMAIN"] = str(args.domain_id)
    os.environ["G1_SLAM_TOPIC"] = args.slam_topic
    if args.slam_open3d:
        os.environ["G1_SLAM_OPEN3D"] = "1"

    stop = threading.Event()

    # ------------------------------------------------  start background jobs
    workers = [
        ("RealSense", threading.Thread(target=_rx_realsense, args=(stop,), daemon=True)),
        ("SLAM", threading.Thread(target=_run_slam, args=(stop,), daemon=True)),
        ("G1", threading.Thread(target=_keyboard_thread, args=(stop, args.iface), daemon=True)),
    ]

    for _name, t in workers:
        t.start()

    # ------------------------------------------------  simple OpenCV window
    try:
        cv2 = _import_cv2()

        while not stop.is_set():
            canvas = _compose_canvas()
            if canvas is None:
                time.sleep(0.05)
                continue

            cv2.imshow("Geoff-Stack", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                stop.set()
                break

        cv2.destroyAllWindows()

    finally:
        stop.set()
        for _name, t in workers:
            t.join(timeout=1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
