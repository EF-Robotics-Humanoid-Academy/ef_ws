#!/usr/bin/env python3
"""
create_slam_map.py
==================

Interactive SLAM map creation utility for Unitree G1.

Features:
- Starts G1 SLAM mapping service (api 1801)
- Shows live mapping progress and map preview in a Tk GUI
- Launches safety/keyboard_controller.py for teleop while mapping
- Save map button -> SLAM end mapping (api 1802)
- Reset map button -> close slam (api 1901) then restart mapping
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

from create_map import OccupancyGrid
from slam_map import SlamInfoSubscriber, SlamMapSubscriber, SlamOdomSubscriber, LidarSwitch
from slam_service import SlamOperateClient, SlamResponse

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


class SlamMapCreatorUI:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args

        self.root = tk.Tk()
        self.root.title("G1 SLAM Map Creator")
        self.root.geometry("1100x760")

        self.slam_client: SlamOperateClient | None = None
        self.slam_sub: SlamMapSubscriber | None = None
        self.odom_sub: SlamOdomSubscriber | None = None
        self.info_sub: SlamInfoSubscriber | None = None
        self.latest_grid: OccupancyGrid | None = None

        self.mapping_active = False
        self.map_start_ts = 0.0
        self.last_map_ts = 0.0
        self.total_updates = 0
        self.changed_cells_total = 0
        self.prev_grid: np.ndarray | None = None
        self.last_info_payload = ""
        self.last_key_payload = ""
        self.teleop_proc: subprocess.Popen[str] | None = None
        self.last_teleop_poll = 0.0

        self.status_var = tk.StringVar(value="Initializing...")
        self.elapsed_var = tk.StringVar(value="Elapsed: 0.0s")
        self.map_var = tk.StringVar(value="Map: waiting")
        self.pose_var = tk.StringVar(value="Pose: waiting")
        self.delta_var = tk.StringVar(value="Update delta: 0 cells")
        self.health_var = tk.StringVar(value="DDS: waiting")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.save_path_var = tk.StringVar(value=args.save_path)

        self._build_ui()
        self._init_backend()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(self.args.update_ms, self._periodic_update)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.status_var, width=95).grid(row=0, column=1, sticky="w", columnspan=6)

        ttk.Label(top, textvariable=self.elapsed_var).grid(row=1, column=0, sticky="w")
        ttk.Label(top, textvariable=self.map_var).grid(row=1, column=1, sticky="w", padx=8)
        ttk.Label(top, textvariable=self.pose_var).grid(row=1, column=2, sticky="w", padx=8)
        ttk.Label(top, textvariable=self.delta_var).grid(row=1, column=3, sticky="w", padx=8)
        ttk.Label(top, textvariable=self.health_var).grid(row=1, column=4, sticky="w", padx=8)

        ttk.Label(top, text="Progress (time target):").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Progressbar(
            top,
            variable=self.progress_var,
            mode="determinate",
            maximum=100.0,
            length=640,
        ).grid(row=2, column=1, columnspan=6, sticky="we", pady=(8, 0))

        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="Map save path (.pcd):").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.save_path_var, width=60).pack(side=tk.LEFT, padx=6)

        ttk.Button(controls, text="Save Map", command=self._save_map).pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="Reset Map", command=self._reset_map).pack(side=tk.LEFT, padx=4)
        self.teleop_btn = ttk.Button(controls, text="Start Teleop", command=self._toggle_teleop)
        self.teleop_btn.pack(side=tk.LEFT, padx=12)

        self.canvas_w = 760
        self.canvas_h = 500
        self.map_canvas = tk.Canvas(self.root, width=self.canvas_w, height=self.canvas_h, bg="#111111", highlightthickness=0)
        self.map_canvas.pack(padx=8, pady=(0, 8), fill=tk.BOTH, expand=False)

        logs = ttk.Frame(self.root, padding=8)
        logs.pack(fill=tk.BOTH, expand=True)
        ttk.Label(logs, text="Logs").pack(anchor="w")
        self.log_text = tk.Text(logs, height=14, wrap="word", bg="#0f1116", fg="#d9e2ec")
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _init_backend(self) -> None:
        self._init_channel_factory()

        self.slam_sub = SlamMapSubscriber(self.args.slam_map_topic)
        self.slam_sub.start()
        self.odom_sub = SlamOdomSubscriber(self.args.slam_odom_topic)
        self.odom_sub.start()
        self.info_sub = SlamInfoSubscriber(self.args.slam_info_topic, self.args.slam_key_topic)
        self.info_sub.start()

        if not self.args.no_lidar_switch:
            try:
                LidarSwitch().set("ON")
                self._log("Published rt/utlidar/switch = ON")
            except Exception as exc:
                self._log(f"WARN: failed to publish lidar switch ON: {exc}")

        self.slam_client = SlamOperateClient()
        self.slam_client.Init()
        self.slam_client.SetTimeout(5.0)
        self._log("SLAM service client initialized: slam_operate v1.0.0.1")

        self._start_mapping()

    def _init_channel_factory(self) -> None:
        """Initialize DDS and fall back to autodetect if iface init fails."""
        iface = (self.args.iface or "").strip()
        if iface.lower() in {"", "auto", "autodetect", "none"}:
            ChannelFactoryInitialize(self.args.domain_id, None)
            self._log(f"DDS initialized (domain={self.args.domain_id}, iface=autodetect)")
            return

        try:
            ChannelFactoryInitialize(self.args.domain_id, iface)
            self._log(f"DDS initialized (domain={self.args.domain_id}, iface={iface})")
        except Exception as exc:
            self._log(
                "WARN: DDS init failed with explicit interface "
                f"'{iface}' ({exc}). Retrying with autodetect."
            )
            ChannelFactoryInitialize(self.args.domain_id, None)
            self._log(f"DDS initialized (domain={self.args.domain_id}, iface=autodetect)")

    def _decode_resp(self, resp: SlamResponse) -> str:
        payload = self._parse_resp_payload(resp)
        if isinstance(payload, dict):
            succ = payload.get("succeed")
            err = payload.get("errorCode")
            info = payload.get("info", "")
            return f"code={resp.code}, succeed={succ}, errorCode={err}, info={info}"
        return f"code={resp.code}, raw={resp.raw}"

    def _parse_resp_payload(self, resp: SlamResponse) -> dict | None:
        raw = resp.raw
        payload: dict | None = None
        if isinstance(raw, (str, bytes)):
            try:
                payload = json.loads(raw.decode() if isinstance(raw, bytes) else raw)
            except Exception:
                payload = None
        return payload if isinstance(payload, dict) else None

    def _resp_ok(self, resp: SlamResponse) -> bool:
        if resp.code != 0:
            return False
        payload = self._parse_resp_payload(resp)
        if payload is None:
            return True
        if payload.get("succeed") is False:
            return False
        err = payload.get("errorCode")
        if isinstance(err, int) and err != 0:
            return False
        return True

    def _start_mapping(self) -> None:
        if self.slam_client is None:
            return
        resp = self.slam_client.start_mapping("indoor")
        self._log(f"Start mapping (api 1801): {self._decode_resp(resp)}")
        if not self._resp_ok(resp):
            self.mapping_active = False
            self.status_var.set("Start mapping failed (see logs)")
            return
        self.mapping_active = True
        self.map_start_ts = time.time()
        self.last_map_ts = 0.0
        self.prev_grid = None
        self.changed_cells_total = 0
        self.total_updates = 0
        self.status_var.set("Mapping active")

    def _save_map(self) -> None:
        if self.slam_client is None:
            return
        path = self.save_path_var.get().strip()
        if not path:
            self._log("ERROR: save path is empty")
            return
        if not path.endswith(".pcd"):
            self._log("WARN: recommended extension is .pcd")
        if not os.path.isabs(path):
            self._log("WARN: save path should be absolute on robot filesystem (e.g. /home/unitree/test.pcd)")
        elif not path.startswith("/home/unitree/"):
            self._log("WARN: non-standard robot path; prefer /home/unitree/*.pcd")
        resp = self.slam_client.end_mapping(path)
        self._log(f"End mapping + save (api 1802, {path}): {self._decode_resp(resp)}")
        if not self._resp_ok(resp):
            self.status_var.set("Save failed (see logs)")
            return
        self.mapping_active = False
        self.status_var.set("Mapping stopped (save requested)")
        self._log("Save request accepted. The .pcd is written on the robot at the path above.")

    def _reset_map(self) -> None:
        if self.slam_client is None:
            return
        close_resp = self.slam_client.close_slam()
        self._log(f"Close SLAM (api 1901): {self._decode_resp(close_resp)}")
        if not self._resp_ok(close_resp):
            self.status_var.set("Reset failed (close slam error)")
            return
        time.sleep(0.2)
        self.map_canvas.delete("all")
        self.latest_grid = None
        self._start_mapping()
        if self.mapping_active:
            self._log("Map reset complete: cleared preview and restarted mapping")

    def _resolve_keyboard_controller(self) -> tuple[str, str] | None:
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(here, "safety", "keyboard_controller.py"),
            os.path.join(here, "..", "safety", "keyboard_controller.py"),
        ]
        for p in candidates:
            if os.path.isfile(p):
                return p, os.path.dirname(p)
        return None

    def _toggle_teleop(self) -> None:
        if self.teleop_proc is not None and self.teleop_proc.poll() is None:
            self.teleop_proc.terminate()
            self._log("Teleop stop requested")
            return

        resolved = self._resolve_keyboard_controller()
        if resolved is None:
            self._log("ERROR: safety/keyboard_controller.py not found")
            return

        script_path, script_dir = resolved
        cmd = [
            sys.executable,
            script_path,
            "--iface",
            self.args.iface,
            "--input",
            self.args.teleop_input,
        ]
        try:
            self.teleop_proc = subprocess.Popen(cmd, cwd=script_dir)
            self.teleop_btn.configure(text="Stop Teleop")
            self._log(
                "Started teleop using safety/keyboard_controller.py. "
                "Use its terminal window controls (WASD/QE, Space, Z, Esc)."
            )
        except Exception as exc:
            self._log(f"ERROR: failed to start teleop: {exc}")

    def _draw_map(self, grid: OccupancyGrid, pose: tuple[float, float, float] | None) -> None:
        g = grid.grid
        h, w = g.shape
        if h <= 0 or w <= 0:
            return

        self.map_canvas.delete("all")
        cell_w = self.canvas_w / float(w)
        cell_h = self.canvas_h / float(h)

        stride = max(1, int(math.ceil(max(w, h) / 220.0)))
        obs = np.argwhere(g > 0)
        if stride > 1:
            obs = obs[::stride]

        for row, col in obs:
            x0 = col * cell_w
            y0 = (h - 1 - row) * cell_h
            x1 = x0 + max(1.0, cell_w * stride)
            y1 = y0 + max(1.0, cell_h * stride)
            self.map_canvas.create_rectangle(x0, y0, x1, y1, fill="#2ea8ff", outline="", tags="map")

        if pose is not None:
            x, y, yaw = pose
            row, col = grid.world_to_grid(x, y)
            px = col * cell_w
            py = (h - 1 - row) * cell_h
            rr = 4
            self.map_canvas.create_oval(px - rr, py - rr, px + rr, py + rr, fill="#00ff99", outline="")
            hx = px + 16 * math.cos(yaw)
            hy = py - 16 * math.sin(yaw)
            self.map_canvas.create_line(px, py, hx, hy, fill="#00ff99", width=2)

    def _log(self, msg: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {msg}\n"
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _teleop_health_update(self) -> None:
        if self.teleop_proc is None:
            self.teleop_btn.configure(text="Start Teleop")
            return
        code = self.teleop_proc.poll()
        if code is None:
            self.teleop_btn.configure(text="Stop Teleop")
            return
        self._log(f"Teleop process exited with code {code}")
        self.teleop_proc = None
        self.teleop_btn.configure(text="Start Teleop")

    def _summarize_slam_json(self, payload: str, source: str) -> None:
        try:
            data = json.loads(payload)
        except Exception:
            return
        ptype = data.get("type", "?")
        err = data.get("errorCode", "?")
        info = data.get("info", "")
        if source == "info":
            if ptype == "pos_info":
                pose = data.get("data", {}).get("currentPose", {})
                self._log(
                    f"slam_info pos_info: err={err}, x={pose.get('x', 0):.2f}, y={pose.get('y', 0):.2f}, info={info}"
                )
            elif ptype == "ctrl_info":
                ctrl = data.get("data", {})
                self._log(
                    f"slam_info ctrl_info: err={err}, arrived={ctrl.get('is_arrived')}, info={info}"
                )
            else:
                self._log(f"slam_info {ptype}: err={err}, info={info}")
        else:
            target = data.get("data", {}).get("targetNodeName")
            arrived = data.get("data", {}).get("is_arrived")
            self._log(f"slam_key_info {ptype}: err={err}, target={target}, arrived={arrived}, info={info}")

    def _periodic_update(self) -> None:
        try:
            if self.slam_sub is not None:
                occ, meta = self.slam_sub.to_occupancy(
                    height_threshold=self.args.height_threshold,
                    max_height=self.args.max_height,
                    origin_centered=self.args.origin_centered,
                )
                if occ is not None and meta is not None:
                    self.latest_grid = occ
                    total_cells = occ.grid.size
                    obstacle_cells = int(np.count_nonzero(occ.grid))
                    pct = 100.0 * obstacle_cells / max(1, total_cells)
                    if meta.timestamp > self.last_map_ts:
                        self.total_updates += 1
                        if self.prev_grid is not None and self.prev_grid.shape == occ.grid.shape:
                            changed = int(np.count_nonzero(self.prev_grid != occ.grid))
                        else:
                            changed = obstacle_cells
                        self.changed_cells_total += changed
                        self.prev_grid = occ.grid.copy()
                        self.last_map_ts = meta.timestamp
                        if self.total_updates % 5 == 1:
                            self._log(
                                f"Map update #{self.total_updates}: "
                                f"{occ.width_cells}x{occ.height_cells}, "
                                f"obs={obstacle_cells}, changed={changed}"
                            )
                    avg_delta = self.changed_cells_total / max(1, self.total_updates)
                    self.map_var.set(
                        f"Map: {occ.width_cells}x{occ.height_cells}, res={occ.resolution:.3f}m, obs={pct:.1f}%"
                    )
                    self.delta_var.set(f"Update delta: avg {avg_delta:.1f} changed cells/frame")

                    pose = None
                    if self.odom_sub is not None and not self.odom_sub.is_stale(max_age=1.2):
                        pose = self.odom_sub.get_pose()
                        self.pose_var.set(f"Pose: x={pose[0]:+.2f}, y={pose[1]:+.2f}, yaw={math.degrees(pose[2]):+.1f}deg")
                    else:
                        self.pose_var.set("Pose: stale/no data")
                    self._draw_map(occ, pose)
                else:
                    self.map_var.set("Map: waiting for rt/utlidar/map_state")

            if self.mapping_active:
                elapsed = time.time() - self.map_start_ts
                self.elapsed_var.set(f"Elapsed: {elapsed:.1f}s")
                progress = min(100.0, 100.0 * elapsed / max(1.0, self.args.target_seconds))
                self.progress_var.set(progress)
            else:
                self.elapsed_var.set("Elapsed: stopped")

            if self.info_sub is not None:
                info = self.info_sub.get_info()
                key = self.info_sub.get_key()
                if info and info != self.last_info_payload:
                    self.last_info_payload = info
                    self._summarize_slam_json(info, "info")
                if key and key != self.last_key_payload:
                    self.last_key_payload = key
                    self._summarize_slam_json(key, "key")

            stale_bits = []
            if self.slam_sub is not None and self.slam_sub.is_stale(1.5):
                stale_bits.append("map_state stale")
            if self.odom_sub is not None and self.odom_sub.is_stale(1.5):
                stale_bits.append("odom stale")
            self.health_var.set("DDS: OK" if not stale_bits else ("DDS: " + ", ".join(stale_bits)))

            now = time.time()
            if now - self.last_teleop_poll > 0.5:
                self.last_teleop_poll = now
                self._teleop_health_update()
        except Exception as exc:
            self._log(f"ERROR in periodic update: {exc}")
            self.status_var.set("Error (see logs)")

        self.root.after(self.args.update_ms, self._periodic_update)

    def _on_close(self) -> None:
        try:
            if self.teleop_proc is not None and self.teleop_proc.poll() is None:
                self.teleop_proc.terminate()
                try:
                    self.teleop_proc.wait(timeout=2.0)
                except Exception:
                    self.teleop_proc.kill()
        except Exception:
            pass

        try:
            if self.slam_client is not None and self.args.close_on_exit:
                resp = self.slam_client.close_slam()
                self._log(f"Close SLAM on exit (api 1901): {self._decode_resp(resp)}")
        except Exception as exc:
            self._log(f"WARN: close slam on exit failed: {exc}")

        self.root.destroy()

    def run(self) -> None:
        self._log(
            f"Started GUI with iface={self.args.iface}, domain_id={self.args.domain_id}, "
            f"save_path={self.args.save_path}"
        )
        self._log("NOTE: save path is on robot storage, not this workstation.")
        self._log("Use 'Start Teleop' and drive the robot to build map coverage.")
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Create and manage G1 SLAM maps with live GUI and keyboard teleop integration.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--iface", default="eth0", help="Network interface in 192.168.123.x segment")
    p.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    p.add_argument("--slam-map-topic", default="rt/utlidar/map_state", help="SLAM map DDS topic")
    p.add_argument("--slam-odom-topic", default="rt/unitree/slam_mapping/odom", help="SLAM odom DDS topic")
    p.add_argument("--slam-info-topic", default="rt/slam_info", help="SLAM info DDS topic")
    p.add_argument("--slam-key-topic", default="rt/slam_key_info", help="SLAM key info DDS topic")
    p.add_argument("--height-threshold", type=float, default=0.15, help="Obstacle threshold for HeightMap")
    p.add_argument("--max-height", type=float, default=None, help="Optional max height clamp")
    p.add_argument("--origin-centered", action="store_true", help="Center map origin around (0,0)")
    p.add_argument("--save-path", default="/home/unitree/test1.pcd", help="Default map save path on robot used by Save Map")
    p.add_argument("--target-seconds", type=float, default=180.0, help="Soft mapping time target shown in progress bar")
    p.add_argument("--update-ms", type=int, default=300, help="GUI update period in milliseconds")
    p.add_argument("--teleop-input", choices=("pynput", "curses"), default="pynput", help="keyboard_controller input backend")
    p.add_argument("--no-lidar-switch", action="store_true", help="Do not publish rt/utlidar/switch ON")
    p.add_argument("--close-on-exit", action="store_true", help="Call SLAM close api (1901) when GUI exits")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    ui = SlamMapCreatorUI(args)
    ui.run()


if __name__ == "__main__":
    main()
