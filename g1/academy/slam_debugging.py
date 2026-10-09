"""Verbose SLAM/LiDAR debugging wrapper around sdk_wrapper.G1.

Import G1 from here exactly like you import it from sdk_wrapper -- it is a
drop-in subclass with the same constructor and the same public API.
start_mapping()/stop_mapping()/relocate() behave identically but print a
detailed trace of what is actually happening, and inspect_slam() is a new,
read-only health check for everything those three depend on.

Why this exists: sdk_wrapper.G1's start_mapping()/stop_mapping()/relocate()
return only {"code": code, "raw": raw}. `code` is the RPC transport status
(0 = the call reached the slam_operate service and got a reply at all); the
service's actual answer -- success/failure, an error code, a human-readable
reason -- is a second JSON payload sitting inside `raw` that sdk_wrapper.G1
never decodes (see _parse_slam_status in sdk_wrapper.py). So `code == 0`
("success=True") only ever tells you the RPC round-tripped, not that SLAM
did what you asked -- which is exactly the gap between "start_mapping said
success" and "relocate silently fails/hangs with no indication why." This
module decodes and prints that second payload on every call, reports
service ON/OFF/PROTECTED status, reports topic freshness (a stale or silent
topic looks identical to a working one in `code`), and for relocate()
specifically prints a live heartbeat while the call is in flight so a
"frozen" relocate() is distinguishable from one that is merely slow.

Usage:
    from slam_debugging import G1
    g1 = G1("eth0")
    g1.inspect_slam()             # read-only health check, run this first
    g1.start_mapping()
    ...
    g1.stop_mapping()
    g1.relocate()
"""
from __future__ import annotations

import threading
import time

import sdk_wrapper

try:
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_ as _LidarImu
except Exception:
    _LidarImu = None

# RPC-level status codes (unitree_sdk2py.rpc.client.Client._Call's `code`),
# copied from modules/scripts/service_view.py's ERROR_HINTS so this file has
# no extra import dependency beyond sdk_wrapper.
RPC_ERROR_HINTS = {
    0: "success",
    3001: "RPC unknown error",
    3102: "RPC client send error",
    3103: "RPC API not registered",
    3104: "RPC timeout",
    3105: "RPC API mismatch",
    3106: "RPC client data error",
    3201: "RPC server send error",
    3202: "RPC server internal error",
    3203: "RPC API not implemented",
    3204: "RPC server parameter error",
    5201: "service switch execution error",
    5202: "service is protected",
}
SERVICE_STATUS_TEXT = {0: "ON", 1: "OFF", 5: "PROTECTED"}

# Raw sensor topics inspect_slam() checks in addition to what sdk_wrapper.G1
# already subscribes to (rt/slam_info, rt/slam_key_info,
# rt/unitree/slam_mapping/odom, rt/odom, SLAM_POINT_TOPICS). These are the
# *unprocessed* LiDAR feed -- checking them separately from the SLAM
# service's own topics tells you whether the hardware itself is streaming,
# independent of whether the SLAM software stack is doing anything with it.
LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_deskewed"
LIDAR_CLOUD_FALLBACK_TOPIC = "rt/utlidar/cloud_livox_mid360"
LIDAR_IMU_TOPIC = "rt/utlidar/imu_livox_mid360"
LIDAR_MAP_TOPIC = "rt/utlidar/map_state"

STALE_AFTER_S = 5.0


def _log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}] [{tag}] {msg}")


def _rpc_hint(code):
    try:
        return RPC_ERROR_HINTS.get(int(code), "unknown RPC code")
    except (TypeError, ValueError):
        return "unparseable RPC code"


def _topic_age_s(latest):
    """Seconds since `latest` (a sdk_wrapper._Latest) last received a
    message, or None if it never has."""
    _msg, ts = latest.get()
    return None if ts <= 0 else max(0.0, time.time() - ts)


def _topic_status(latest):
    age = _topic_age_s(latest)
    if age is None:
        return "WARN", "never received"
    if age > STALE_AFTER_S:
        return "WARN", f"stale ({age:.1f}s since last message)"
    return "OK", f"fresh ({age:.2f}s since last message)"


class G1(sdk_wrapper.G1):
    """sdk_wrapper.G1 with verbose start_mapping/stop_mapping/relocate
    logging and a read-only inspect_slam() health check. Same constructor,
    same public API -- swap the import and nothing else changes."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lidar_cloud_dbg = self._latest(LIDAR_CLOUD_TOPIC, sdk_wrapper.PointCloud2_)
        self._lidar_cloud_fallback_dbg = self._latest(LIDAR_CLOUD_FALLBACK_TOPIC, sdk_wrapper.PointCloud2_)
        self._lidar_map_dbg = self._latest(LIDAR_MAP_TOPIC, sdk_wrapper.String_)
        self._lidar_imu_dbg = self._latest(LIDAR_IMU_TOPIC, _LidarImu) if _LidarImu is not None else None

    # -- shared helpers ------------------------------------------------

    def _service_status_text(self, name):
        try:
            row = self.get_service(name)
        except Exception as exc:
            return "FAIL", f"ServiceList RPC failed: {exc!r}"
        if row is None:
            return "WARN", "not reported by ServiceList (older firmware?)"
        status = row.get("status")
        text = SERVICE_STATUS_TEXT.get(status, f"unknown({status})")
        if row.get("protected"):
            text += " (protected)"
        level = "OK" if status == 0 else ("WARN" if status == 5 else "FAIL")
        return level, text

    def _freshest_slam_topic_text(self):
        best_topic, best_age = None, None
        for topic, latest in (("rt/slam_info", self._slam_info), ("rt/slam_key_info", self._slam_key)):
            age = _topic_age_s(latest)
            if age is not None and (best_age is None or age < best_age):
                best_topic, best_age = topic, age
        if best_topic is None:
            return "no rt/slam_info or rt/slam_key_info message received yet"
        return f"{best_topic} {best_age:.2f}s ago"

    def _decode_and_log(self, tag, result, elapsed_s=None):
        code = result.get("code")
        raw = result.get("raw")
        status = sdk_wrapper._parse_slam_status(raw) or {}
        result["error_code"] = status.get("error_code")
        result["info"] = status.get("info")
        result["is_arrived"] = status.get("is_arrived")
        result["obstacle_blocked"] = status.get("obstacle_blocked")
        result["ok"] = (int(code) == 0) and (int(status.get("error_code") or 0) == 0)
        elapsed_txt = f" in {elapsed_s:.2f}s" if elapsed_s is not None else ""
        _log(tag, f"RPC code={code} ({_rpc_hint(code)}){elapsed_txt}")
        if raw is None:
            _log(tag, "raw response: <none -- the RPC timed out or the service never replied>")
        else:
            _log(tag, f"raw response: {raw}")
            _log(tag, f"decoded payload: errorCode={status.get('error_code')} info={status.get('info')!r} "
                      f"is_arrived={status.get('is_arrived')} obstacle_blocked={status.get('obstacle_blocked')}")
        _log(tag, f"=== {tag.lower()} {'OK' if result['ok'] else 'FAILED'} ===")
        return result

    # -- overrides -------------------------------------------------------

    def start_mapping(self, slam_type="indoor"):
        _log("START_MAPPING", f"=== start_mapping(slam_type={slam_type!r}) ===")
        level, text = self._service_status_text("unitree_slam")
        _log("START_MAPPING", f"service unitree_slam: {text}")
        if level == "FAIL":
            _log("START_MAPPING", "unitree_slam is not ON -- the RPC below will likely fail or be ignored")
        t0 = time.time()
        result = super().start_mapping(slam_type)
        return self._decode_and_log("START_MAPPING", result, time.time() - t0)

    def stop_mapping(self, save_path=None, save=True):
        _log("STOP_MAPPING", f"=== stop_mapping(save_path={save_path!r}, save={save}) ===")
        _log("STOP_MAPPING", f"map address that will be used: {save_path or self._slam_map_path!r}")
        t0 = time.time()
        result = super().stop_mapping(save_path=save_path, save=save)
        elapsed = time.time() - t0
        if not save:
            return self._decode_and_log("STOP_MAPPING", result, elapsed)
        result = self._decode_and_log("STOP_MAPPING", result, elapsed)
        if result["ok"]:
            _log("STOP_MAPPING", f"map saved to (mainboard path) {self._slam_map_path!r} -- "
                                  "this is also the address relocate() will load from")
        return result

    def relocate(self, map_path=None, pose=None):
        _log("RELOCATE", "=== relocate() ===")
        if pose is not None:
            _log("RELOCATE", f"using explicit pose={pose}")
        else:
            resolved, source = self._slam_pose(), "current SLAM pose (rt/slam_info)"
            if resolved is None:
                resolved, source = self._last_slam_pose, "last-known SLAM pose"
            if resolved is None:
                resolved, source = self._initial_slam_pose, "initial SLAM pose (captured at start_mapping())"
            if resolved is None:
                resolved, source = (0.0, 0.0, 0.0), "hardcoded (0, 0, 0) -- no SLAM pose has EVER been observed"
            _log("RELOCATE", f"no pose given -- resolved to {resolved} via {source}")
            if source.startswith("hardcoded"):
                _log("RELOCATE", "WARNING: relocating to (0, 0, 0) blind is very likely wrong "
                                  "unless that really is where the robot is on the saved map")
        map_path_used = str(map_path) if map_path else self._slam_map_path
        _log("RELOCATE", f"map address: {map_path_used!r}")
        level, text = self._service_status_text("unitree_slam")
        _log("RELOCATE", f"service unitree_slam: {text}")

        stop = threading.Event()

        def _heartbeat():
            start = time.time()
            while not stop.wait(2.0):
                _log("RELOCATE", f"...still waiting on the init_pose RPC "
                                  f"({time.time() - start:.1f}s elapsed); "
                                  f"most recent SLAM topic update: {self._freshest_slam_topic_text()}")

        hb = threading.Thread(target=_heartbeat, name="relocate-heartbeat", daemon=True)
        hb.start()
        t0 = time.time()
        try:
            result = super().relocate(map_path=map_path, pose=pose)
        finally:
            stop.set()
            hb.join(timeout=1.0)
        return self._decode_and_log("RELOCATE", result, time.time() - t0)

    def get_slam_pose(self):
        pose = super().get_slam_pose()
        if pose is None:
            _log("GET_SLAM_POSE", "returning None -- no rt/slam_info pose parsed successfully "
                                   f"and no cached last-known pose either. {self._freshest_slam_topic_text()}")
        return pose

    # -- new: read-only health check --------------------------------------

    def inspect_slam(self, verbose=True):
        """Read-only health check for everything start_mapping()/
        stop_mapping()/relocate() depend on. Makes NO state-changing RPC
        calls (no start/stop/pause/pose-nav) -- only reads already-
        subscribed DDS topics plus the also-read-only ServiceList RPC, so
        it is safe to run at any time, including mid-mapping or
        mid-navigation.

        Returns {"ok": bool, "checks": [{"name", "level", "detail"}, ...]}.
        Prints a human-readable report when verbose=True (the default)."""
        checks = []

        def add(name, level, detail):
            checks.append({"name": name, "level": level, "detail": detail})
            if verbose:
                _log(level, f"{name}: {detail}")

        if verbose:
            _log("INSPECT", "=== SLAM/LiDAR health check ===")

        for service in ("unitree_slam", "ai_sport"):
            level, text = self._service_status_text(service)
            add(f"service:{service}", level, text)

        topic_checks = [
            ("rt/slam_info", self._slam_info),
            ("rt/slam_key_info", self._slam_key),
            ("rt/unitree/slam_mapping/odom", self._slam_odom),
            ("rt/odom", self._odom),
        ]
        topic_checks += list(zip(sdk_wrapper.SLAM_POINT_TOPICS, [latest for _topic, latest in self._clouds]))
        topic_checks += [
            (LIDAR_CLOUD_TOPIC, self._lidar_cloud_dbg),
            (LIDAR_CLOUD_FALLBACK_TOPIC, self._lidar_cloud_fallback_dbg),
            (LIDAR_MAP_TOPIC, self._lidar_map_dbg),
        ]
        for topic, latest in topic_checks:
            level, detail = _topic_status(latest)
            add(f"topic:{topic}", level, detail)
        if self._lidar_imu_dbg is None:
            add(f"topic:{LIDAR_IMU_TOPIC}", "WARN", "Imu_ type unavailable in this unitree_sdk2py install -- skipped")
        else:
            level, detail = _topic_status(self._lidar_imu_dbg)
            add(f"topic:{LIDAR_IMU_TOPIC}", level, detail)

        raw = self.get_slam_info()
        if raw is None:
            add("slam_info:payload", "WARN", "no rt/slam_info or rt/slam_key_info message received yet")
        else:
            status = sdk_wrapper._parse_slam_status(raw) or {}
            pose = sdk_wrapper._parse_slam_pose(raw)
            level = "OK" if int(status.get("error_code") or 0) == 0 else "FAIL"
            add("slam_info:payload", level,
                f"errorCode={status.get('error_code')} info={status.get('info')!r} "
                f"is_arrived={status.get('is_arrived')} obstacle_blocked={status.get('obstacle_blocked')} "
                f"pose={pose}")

        ok = all(c["level"] != "FAIL" for c in checks)
        if verbose:
            n_fail = sum(1 for c in checks if c["level"] == "FAIL")
            n_warn = sum(1 for c in checks if c["level"] == "WARN")
            _log("INSPECT", f"=== {len(checks)} checks: {n_fail} FAIL, {n_warn} WARN "
                             f"-- overall {'OK' if ok else 'FAIL'} ===")
        return {"ok": ok, "checks": checks}


if __name__ == "__main__":
    import sys
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    domain_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    g1 = G1(iface, domain_id=domain_id)
    time.sleep(1.0)  # give DDS subscribers a moment to receive their first message
    g1.inspect_slam()
