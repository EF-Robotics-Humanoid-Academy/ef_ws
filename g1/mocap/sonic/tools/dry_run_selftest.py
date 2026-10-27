#!/usr/bin/env python3
"""Automated dry-run validation of the teleop safety layer.

Runs bin/g1_deploy_onnx_ref in --dry-run on the loopback interface (it cannot
reach the robot: no LowCmd publisher is created and DDS stays on `lo`) against
a local synthetic mocap streamer, drives the operator keys, injects faults and
checks the per-tick CSV.  On success writes logs/teleop_safety/VALIDATED, which
`./teleop_safe.sh real` requires for the same config file and binary.

Usage:  python3 tools/dry_run_selftest.py [--config config/teleop_safety.conf] [--model release]
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_mocap_streamer import R_SHOULDER_PITCH, Streamer  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BINARY = os.path.join(ROOT, "bin", "g1_deploy_onnx_ref")
PORT = 5556


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class Results:
    def __init__(self):
        self.items = []

    def check(self, scenario, ok, what, detail=""):
        self.items.append({"scenario": scenario, "ok": bool(ok), "check": what, "detail": detail})
        print(f"  [{'PASS' if ok else 'FAIL'}] {what}" + (f"  ({detail})" if detail else ""))
        return ok

    @property
    def passed(self):
        return all(i["ok"] for i in self.items)


class Controller:
    """Runs the controller binary with a piped stdin and captured output."""

    def __init__(self, args, log_path, stdin_text=None):
        arch = os.uname().machine
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = ":".join([
            f"{ROOT}/thirdparty/unitree_sdk2/thirdparty/lib/{arch}",
            f"{ROOT}/thirdparty/onnxruntime/lib",
            f"{env.get('TensorRT_ROOT', '/usr')}/lib",
            env.get("LD_LIBRARY_PATH", ""),
        ])
        self.t0 = time.monotonic()
        self.lines = []
        self.log = open(log_path, "w")
        self.proc = subprocess.Popen([BINARY] + args, cwd=ROOT, env=env, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        if stdin_text is not None:
            self.proc.stdin.write(stdin_text)
            self.proc.stdin.flush()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self._deadman = threading.Event()
        self._closing = threading.Event()
        self.dm_thread = threading.Thread(target=self._deadman_loop, daemon=True)
        self.dm_thread.start()

    def _read(self):
        for line in self.proc.stdout:
            t = time.monotonic() - self.t0
            self.lines.append((t, line.rstrip("\n")))
            self.log.write(f"{t:8.3f} {line}")
            self.log.flush()

    def _deadman_loop(self):
        # Mimics a held key with terminal auto-repeat (~20 Hz).
        while not self._closing.is_set():
            if self._deadman.is_set():
                self.key(" ")
            time.sleep(0.05)

    def key(self, k):
        try:
            self.proc.stdin.write(k)
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def deadman(self, held):
        (self._deadman.set if held else self._deadman.clear)()

    def wait_for(self, text, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in l for _, l in self.lines):
                return True
            if self.proc.poll() is not None:
                time.sleep(0.2)
                return any(text in l for _, l in self.lines)
            time.sleep(0.05)
        return False

    def has(self, text):
        return any(text in l for _, l in self.lines)

    def finish(self, how="O", timeout=20):
        self._deadman.clear()
        if self.proc.poll() is None:
            if how == "O":
                self.key("O")
            elif how == "SIGINT":
                self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._closing.set()
        self.reader.join(timeout=2)
        self.log.close()
        return self.proc.returncode


def base_args(model, config, extra_sets):
    args = ["lo", f"policy/{model}/model_decoder.onnx", "reference/",
            "--obs-config", f"policy/{model}/observation_config.yaml",
            "--encoder-file", f"policy/{model}/model_encoder.onnx",
            "--input-type", "zmq", "--output-type", "zmq",
            "--zmq-host", "localhost", "--zmq-port", str(PORT), "--zmq-topic", "pose",
            "--teleop-safety-config", config]
    for kv in extra_sets:
        args += ["--safety-set", kv]
    return args


def load_csv(path):
    meta, body = {}, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line[1:].strip().split(",")
                if len(parts) > 1:
                    meta[parts[0].strip()] = np.array([float(x) for x in parts[1:]])
            else:
                body.append(line)
    rows = list(csv.DictReader(body))
    col = lambda prefix: np.array([[float(r[f"{prefix}{i}"]) for i in range(29)] for r in rows])
    data = {
        "mode": [r["mode"] for r in rows],
        "temp_state": [r["temp_state"] for r in rows],
        "deadman": np.array([int(r["deadman_ok"]) for r in rows]),
        "fresh": np.array([int(r["fresh"]) for r in rows]),
        "streamed": np.array([int(r["streamed"]) for r in rows]),
        "raw": col("raw"), "ref": col("ref"), "vel": col("vel"), "motor": col("motor_hw"),
    }
    return meta, data


def runs(seq, value):
    out, start = [], None
    for i, v in enumerate(seq + [None]):
        if v == value and start is None:
            start = i
        elif v != value and start is not None:
            out.append((start, i - start))
            start = None
    return out


def check_invariants(res, name, meta, d, dt, scale=1.0):
    vmax, amax = meta["vmax"] * scale, meta["amax"] * scale
    ref, vel, motor = d["ref"], d["vel"], d["motor"]
    finite = np.isfinite(ref).all() and np.isfinite(vel).all() and np.isfinite(motor).all()
    res.check(name, finite, "reference, velocity and motor targets always finite")
    lo_ex = (meta["ref_lo"] - ref).max()
    hi_ex = (ref - meta["ref_hi"]).max()
    res.check(name, max(lo_ex, hi_ex) <= 1e-6, "reference inside URDF limits minus margin",
              f"worst excess {max(lo_ex, hi_ex):+.2e} rad")
    m_ex = max((meta["urdf_hw_lo"] - motor).max(), (motor - meta["urdf_hw_hi"]).max())
    res.check(name, m_ex <= 1e-5, "motor targets inside URDF limits", f"worst excess {m_ex:+.2e} rad")
    dq = np.abs(np.diff(ref, axis=0))
    dv = np.abs(np.diff(vel, axis=0))
    # CSV values are serialised floats; allow 1e-6 absolute for rounding only.
    tol = 1e-6
    q_ratio = (dq / (vmax * dt)).max()
    v_ratio = (dv / (amax * dt)).max()
    res.check(name, (dq <= vmax * dt + tol).all(), "per-cycle reference step <= vmax*dt",
              f"max {dq.max():.4f} rad/cycle = {q_ratio:.3f} of limit")
    res.check(name, (dv <= amax * dt + tol).all(), "per-cycle reference velocity change <= amax*dt",
              f"max {dv.max():.4f} rad/s per cycle = {v_ratio:.3f} of limit")
    return dq.max()


def scenario_main(res, model, config, logdir):
    name = "main"
    print(f"\n=== Scenario '{name}': startup, dead-man, ramp, faults, dropout, stop ===")
    csv_path = os.path.join(logdir, "main.csv")
    streamer = Streamer(PORT).start()
    c = Controller(base_args(model, config, [f"csv_log_path={csv_path}"]) + ["--dry-run"],
                   os.path.join(logdir, "main.log"))
    ok = res.check(name, c.wait_for("Init Done", 240), "controller initialised in dry-run")
    if ok:
        res.check(name, c.has("DRY-RUN: no LowCmd publisher"), "dry-run banner: no LowCmd publisher / ReleaseMode")
        res.check(name, not c.has("Release Mode"), "ReleaseMode never attempted")
        timeline = [
            (0.0, "press ] (start control)", lambda: c.key("]")),
            (1.5, "press Enter (mocap on)", lambda: c.key("\n")),
            (2.5, "dead-man held", lambda: c.deadman(True)),
            (8.0, "abrupt 1.0 rad step on right shoulder pitch", lambda: streamer.set_step(R_SHOULDER_PITCH, -1.0)),
            (11.0, "5 NaN frames", lambda: streamer.inject_nan(5)),
            (12.0, "3 garbage frames (knee = 5 rad)", lambda: streamer.inject_garbage(3)),
            (13.0, "skip 10 frame indices", lambda: streamer.skip_frames(10)),
            (14.0, "stream dropout 600 ms", lambda: streamer.dropout(0.6)),
            (19.5, "root yaw jump 90 deg", lambda: streamer.set_yaw(math.pi / 2)),
            (21.0, "dead-man released", lambda: c.deadman(False)),
            (22.5, "dead-man held again", lambda: c.deadman(True)),
            (27.0, "step removed", lambda: streamer.set_step(R_SHOULDER_PITCH, 0.0)),
            (28.0, "press Enter (mocap off -> preloaded reference)", lambda: c.key("\n")),
        ]
        t_start = time.monotonic()
        for at, label, action in timeline:
            time.sleep(max(0.0, t_start + at - time.monotonic()))
            print(f"    t={at:5.1f}s  {label}")
            action()
        time.sleep(max(0.0, t_start + 31.0 - time.monotonic()))
    print("    t= 31.0s  press O (emergency stop)")
    rc = c.finish("O")
    streamer.stop()
    res.check(name, rc == 0, "process exits cleanly after O", f"exit code {rc}")
    res.check(name, c.has("Stop requested -> damping"), "O -> damping command (existing E-stop behaviour)")
    for text, what in [
        ("Rejected invalid mocap frame", "invalid (NaN / out-of-range) frames rejected and logged"),
        ("Dropped/skipped", "skipped frame indices detected and logged"),
        ("Mocap stale", "mocap timeout detected and logged"),
        ("Dead-man RELEASED", "dead-man release detected"),
        ("Session summary", "session summary printed on shutdown"),
    ]:
        res.check(name, c.has(text), what)
    if not os.path.exists(csv_path):
        res.check(name, False, "CSV log written")
        return
    meta, d = load_csv(csv_path)
    dt = 0.02
    res.check(name, len(d["mode"]) > 1000, "CSV log written", f"{len(d['mode'])} control ticks")
    max_dq = check_invariants(res, name, meta, d, dt)
    raw_step = np.nanmax(np.abs(np.diff(d["raw"], axis=0)))
    res.check(name, raw_step > 0.9, "abrupt raw mocap step present in input", f"max raw jump {raw_step:.3f} rad/cycle")
    print(f"    smoothing: raw input jumped up to {raw_step:.3f} rad in one cycle; reference moved at most {max_dq:.4f} rad")

    modes = d["mode"]
    active = np.array([m in ("RAMP", "FOLLOW") for m in modes])
    res.check(name, not (active & (d["deadman"] == 0)).any(), "never RAMP/FOLLOW without dead-man")
    res.check(name, not (active & (d["fresh"] == 0)).any(), "never RAMP/FOLLOW with stale mocap")
    pre = modes[: next((i for i, m in enumerate(modes) if m != "HOLD"), len(modes))]
    res.check(name, len(pre) > 50 and np.abs(np.diff(d["ref"][: len(pre)], axis=0)).max() < 1e-9,
              "HOLD without dead-man: reference does not move", f"{len(pre)} ticks before first ramp")
    ramps = runs(modes, "RAMP")
    first_len = ramps[0][1] if ramps else 0
    res.check(name, ramps and abs(first_len - 200) <= 2, "first ramp lasts ~4.0 s", f"{first_len * dt:.2f} s")
    res.check(name, len(ramps) >= 3, "re-ramp after dropout, dead-man re-engage and source change",
              f"{len(ramps)} ramps")
    holds = runs(modes, "HOLD")
    stale_holds = [(s, n) for s, n in holds[1:] if n > 5]
    stopped = all(np.abs(d["vel"][s + n - 1]).max() < 1e-6 for s, n in stale_holds)
    res.check(name, stale_holds and stopped, "reference velocity reaches zero in every HOLD",
              f"{len(stale_holds)} hold episodes")
    stale_rows = np.where((d["streamed"] == 1) & (d["fresh"] == 0))[0]
    res.check(name, len(stale_rows) > 0 and all(modes[i] == "HOLD" for i in stale_rows),
              "stale mocap rows are all HOLD", f"{len(stale_rows)} stale ticks")


def scenario_watchdog(res, model, config, logdir):
    name = "watchdog"
    print(f"\n=== Scenario '{name}': injected 300 ms control-loop stall ===")
    streamer = Streamer(PORT).start()
    c = Controller(base_args(model, config, ["dry_run_inject_stall_after_s=2"]) + ["--dry-run"],
                   os.path.join(logdir, "watchdog.log"))
    if res.check(name, c.wait_for("Init Done", 240), "controller initialised"):
        c.key("]")
        res.check(name, c.wait_for("COMMAND WATCHDOG", 15), "watchdog trips on stalled control loop")
        res.check(name, c.wait_for("Session summary", 20), "process shuts down into damping by itself")
    rc = c.finish("SIGINT")
    streamer.stop()
    res.check(name, c.has("watchdog_fault=yes"), "summary reports watchdog fault")
    res.check(name, rc == 0, "clean exit", f"exit code {rc}")


def scenario_temperature(res, model, config, logdir, temp, expect_state):
    name = f"temperature_{int(temp)}C"
    print(f"\n=== Scenario '{name}': synthetic motor temperature {temp} C, stop with SIGINT ===")
    csv_path = os.path.join(logdir, f"{name}.csv")
    streamer = Streamer(PORT).start()
    c = Controller(base_args(model, config, [f"dry_run_synthetic_temperature_c={temp}", f"csv_log_path={csv_path}"])
                   + ["--dry-run"], os.path.join(logdir, f"{name}.log"))
    if res.check(name, c.wait_for("Init Done", 240), "controller initialised"):
        c.key("]")
        time.sleep(0.5)
        c.key("\n")
        time.sleep(0.5)
        c.deadman(True)
        time.sleep(7.0)
    rc = c.finish("SIGINT")
    streamer.stop()
    res.check(name, c.has(f"-> {expect_state}"), f"temperature state {expect_state} entered")
    res.check(name, c.has("Shutdown signal received"), "SIGINT handled through normal Stop() path")
    res.check(name, c.has("Stop requested -> damping"), "SIGINT -> damping")
    res.check(name, rc == 0, "clean exit after SIGINT", f"exit code {rc}")
    if os.path.exists(csv_path):
        meta, d = load_csv(csv_path)
        if expect_state == "STOPPED":
            res.check(name, all(m == "HOLD" for m in d["mode"]), "no motion while over stop temperature (always HOLD)")
        else:
            check_invariants(res, name, meta, d, 0.02, scale=0.5)
            res.check(name, "FOLLOW" in d["mode"] or "RAMP" in d["mode"], "still follows (slower) when reduced")


def scenario_startup_guards(res, model, config, logdir):
    name = "startup_guards"
    print(f"\n=== Scenario '{name}': confirmation prompt and config validation ===")
    c = Controller(base_args(model, config, []), os.path.join(logdir, "confirm_refused.log"), stdin_text="no\n")
    rc = c.finish(how="none", timeout=20)
    res.check(name, c.has("Type ENABLE") and c.has("Not confirmed"), "physical mode requires typing ENABLE")
    res.check(name, not c.has("Creating G1Deploy"), "nothing created (no DDS, no ReleaseMode) when not confirmed")
    res.check(name, rc == 0, "exits cleanly", f"exit code {rc}")

    c = Controller(base_args(model, config, []) + ["--disable-crc-check"], os.path.join(logdir, "crc_refused.log"),
                   stdin_text="ENABLE\n")
    rc = c.finish(how="none", timeout=20)
    res.check(name, rc == 2 and not c.has("Creating G1Deploy"), "--disable-crc-check refused in physical mode")

    c = Controller(base_args(model, config, ["max_velocty_fraction=0.2"]) + ["--dry-run"],
                   os.path.join(logdir, "bad_key.log"))
    rc = c.finish(how="none", timeout=20)
    res.check(name, rc == 2 and c.has("unknown teleop safety key"), "misspelled key rejected")

    c = Controller(base_args(model, config, ["temperature_stop_c=95"]) + ["--dry-run"],
                   os.path.join(logdir, "bad_temp.log"))
    rc = c.finish(how="none", timeout=20)
    res.check(name, rc == 2 and c.has("outside the allowed range"), "raising temperature limit above 90 C rejected")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config/teleop_safety.conf")
    ap.add_argument("--model", default="release", choices=["release", "low_latency"])
    args = ap.parse_args()

    config_abs = os.path.join(ROOT, args.config) if not os.path.isabs(args.config) else args.config
    stamp_dir = os.path.join(ROOT, "logs", "teleop_safety")
    logdir = os.path.join(stamp_dir, "selftest_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(logdir, exist_ok=True)
    print(f"Dry-run selftest  config={config_abs}  model={args.model}  logs={logdir}")
    print("Interface: lo (loopback). No LowCmd publisher is created; nothing can reach the robot.")

    res = Results()
    scenario_startup_guards(res, args.model, args.config, logdir)
    scenario_main(res, args.model, args.config, logdir)
    scenario_watchdog(res, args.model, args.config, logdir)
    scenario_temperature(res, args.model, args.config, logdir, 72, "REDUCED")
    scenario_temperature(res, args.model, args.config, logdir, 87, "STOPPED")

    n_fail = sum(not i["ok"] for i in res.items)
    report = {
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "config": config_abs, "config_sha256": sha256(config_abs),
        "binary_sha256": sha256(BINARY), "model": args.model,
        "checks": len(res.items), "failures": n_fail, "results": res.items,
    }
    with open(os.path.join(logdir, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n{len(res.items)} checks, {n_fail} failures -> {'PASS' if res.passed else 'FAIL'}")
    print(f"Report: {os.path.join(logdir, 'report.json')}")
    stamp = os.path.join(stamp_dir, "VALIDATED")
    if res.passed:
        with open(stamp, "w") as f:
            json.dump({k: report[k] for k in ("timestamp", "config", "config_sha256", "binary_sha256", "model")}
                      | {"report": os.path.join(logdir, "report.json")}, f, indent=2)
        print(f"Validation stamp written: {stamp}")
    elif os.path.exists(stamp):
        os.remove(stamp)
        print("Previous validation stamp removed.")
    return 0 if res.passed else 1


if __name__ == "__main__":
    sys.exit(main())
