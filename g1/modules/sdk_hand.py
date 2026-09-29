from __future__ import annotations
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core import channel as channel_module

import os
import time
import threading
from typing import Any, Dict

from dds_env import (
    default_dds_iface,
    ensure_channel_factory_initialized,
    ensure_cyclonedds_environment,
)

ensure_cyclonedds_environment()


try:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
except Exception:
    HandState_ = None  # type: ignore[assignment]


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
channel_module.ChannelConfigAutoDetermine = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""
os.environ.setdefault(
    "CYCLONEDDS_URI",
    "<CycloneDDS><Domain><Tracing><Category>none</Category></Tracing></Domain></CycloneDDS>",
)


# Flipped: the dex3 hand units are physically installed on the wrong wrists
# (the left-designed hand sits on the right wrist and vice versa), so both
# of the following have to be swapped here to make the API-level "left"/
# "right" side match the anatomical wrist -- see the matching comment in
# academy/sdk_wrapper.py's HAND_CMD_TOPICS for the full rationale:
#  1. TOPIC_HAND_BY_SIDE/HAND_STATE_TOPIC_BY_SIDE -- each hand unit's own DDS
#     topic follows the unit, not the wrist, so this reaches the correct
#     wrist.
#  2. HAND_MAX_LIMITS/HAND_MIN_LIMITS/HAND_THUMB_0_HOLD_TARGETS -- the
#     joint-limit geometry is a property of the hand unit's own (mirrored)
#     design, so this must match whichever unit that wrist's topic now
#     points at. Everything below (HAND_CLOSED_LIMITS/HAND_OPEN_LIMITS/
#     clamp_hand_targets) derives from these, so swapping only these five
#     dicts keeps it all self-consistent.
TOPIC_HAND_BY_SIDE = {
    "left": "rt/dex3/right/cmd",
    "right": "rt/dex3/left/cmd",
}

HAND_STATE_TOPIC_BY_SIDE = {
    "left": "rt/dex3/right/state",
    "right": "rt/dex3/left/state",
}

HAND_MAX_LIMITS = {
    "left": [1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75],
    "right": [1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0],
}

HAND_MIN_LIMITS = {
    "left": [-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0],
    "right": [-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75],
}

HAND_THUMB_0_HOLD_TARGETS = {
    "left": -0.03510913997888565,
    "right": -0.09927542507648468,
}

# Backwards-compatible right-hand presets. New code should use
# hand_open_targets(), hand_closed_targets(), or hand_grip_targets().
HAND_OPEN = [
    HAND_THUMB_0_HOLD_TARGETS["right"],
    HAND_MAX_LIMITS["right"][1],
    HAND_MAX_LIMITS["right"][2],
    HAND_MIN_LIMITS["right"][3],
    HAND_MIN_LIMITS["right"][4],
    HAND_MIN_LIMITS["right"][5],
    HAND_MIN_LIMITS["right"][6],
]

HAND_CLOSED = [
    HAND_THUMB_0_HOLD_TARGETS["right"],
    HAND_MIN_LIMITS["right"][1],
    HAND_MIN_LIMITS["right"][2],
    HAND_MAX_LIMITS["right"][3],
    HAND_MAX_LIMITS["right"][4],
    HAND_MAX_LIMITS["right"][5],
    HAND_MAX_LIMITS["right"][6],
]

HAND_CLOSED_LIMITS = {
    "left": [
        HAND_THUMB_0_HOLD_TARGETS["left"],
        HAND_MAX_LIMITS["left"][1],
        HAND_MAX_LIMITS["left"][2],
        HAND_MIN_LIMITS["left"][3],
        HAND_MIN_LIMITS["left"][4],
        HAND_MIN_LIMITS["left"][5],
        HAND_MIN_LIMITS["left"][6],
    ],
    "right": [
        HAND_THUMB_0_HOLD_TARGETS["right"],
        HAND_MIN_LIMITS["right"][1],
        HAND_MIN_LIMITS["right"][2],
        HAND_MAX_LIMITS["right"][3],
        HAND_MAX_LIMITS["right"][4],
        HAND_MAX_LIMITS["right"][5],
        HAND_MAX_LIMITS["right"][6],
    ],
}

HAND_OPEN_LIMITS = {
    side: [
        closed[0],
        *[
            hi if abs(closed_value - lo) < abs(closed_value - hi) else lo
            for closed_value, lo, hi in zip(
                closed[1:],
                HAND_MIN_LIMITS[side][1:],
                HAND_MAX_LIMITS[side][1:],
            )
        ],
    ]
    for side, closed in HAND_CLOSED_LIMITS.items()
}

FINGER_TO_IDXS: Dict[str, list[int]] = {
    "thumb": [0, 1, 2],
    "middle": [3, 4],
    "index": [5, 6],
}


def hand_mid_targets(hand: str) -> list[float]:
    side = str(hand).strip().lower()
    return [
        (lo + hi) / 2.0
        for lo, hi in zip(HAND_MIN_LIMITS[side], HAND_MAX_LIMITS[side])
    ]


def _with_thumb_0(targets: list[float], thumb_0: float | None) -> list[float]:
    if thumb_0 is not None:
        targets[0] = float(thumb_0)
    return targets


def hand_open_targets(hand: str, thumb_0: float | None = None) -> list[float]:
    side = str(hand).strip().lower()
    return _with_thumb_0(list(HAND_OPEN_LIMITS[side]), thumb_0)


def hand_closed_targets(hand: str, thumb_0: float | None = None) -> list[float]:
    side = str(hand).strip().lower()
    return _with_thumb_0(list(HAND_CLOSED_LIMITS[side]), thumb_0)


def hand_grip_targets(hand: str, percent: float, thumb_0: float | None = None) -> list[float]:
    alpha = min(1.0, max(0.0, float(percent) / 100.0))
    open_targets = hand_open_targets(hand, thumb_0=thumb_0)
    closed_targets = hand_closed_targets(hand, thumb_0=thumb_0)
    return [
        start + (stop - start) * alpha
        for start, stop in zip(open_targets, closed_targets)
    ]


def clamp_hand_targets(hand: str, targets: list[float]) -> list[float]:
    side = str(hand).strip().lower()
    if len(targets) != 7:
        raise ValueError("Hand targets must contain 7 joint values.")
    return [
        max(float(lo), min(float(hi), float(value)))
        for value, lo, hi in zip(targets, HAND_MIN_LIMITS[side], HAND_MAX_LIMITS[side])
    ]


def pack_ris_mode(motor_id: int, status: int = 1, timeout: int = 0) -> int:
    return (
        (int(motor_id) & 0x0F)
        | ((int(status) & 0x07) << 4)
        | ((int(timeout) & 0x01) << 7)
    )


def build_hand_msg(
    targets: list[float],
    kp: float,
    kd: float,
    tau: float,
    *,
    timeout: int = 0,
) -> HandCmd_:
    if len(targets) != 7:
        raise ValueError("Hand targets must contain 7 joint values.")
    msg = unitree_hg_msg_dds__HandCmd_()
    for idx in range(7):
        cmd = msg.motor_cmd[idx]
        cmd.mode = pack_ris_mode(idx, timeout=timeout)
        cmd.tau = float(tau)
        cmd.q = float(targets[idx])
        cmd.dq = 0.0
        cmd.kp = float(kp)
        cmd.kd = float(kd)
    return msg


class Dex3HandController:
    def __init__(self, hand: str = "right", iface: str | None = None, domain_id: int = 0) -> None:
        side = str(hand).strip().lower()
        if side not in TOPIC_HAND_BY_SIDE:
            raise ValueError(f"Invalid hand '{hand}'.")
        self.hand = side
        self.iface = str(iface) if iface is not None else default_dds_iface("eth0")
        self.domain_id = int(domain_id)
        ensure_channel_factory_initialized(self.domain_id, self.iface)
        self._pub = ChannelPublisher(TOPIC_HAND_BY_SIDE[self.hand], HandCmd_)
        self._pub.Init()
        self._last_targets: list[float] | None = None
        self._release_stop: threading.Event | None = None
        self._release_thread: threading.Thread | None = None
        self._cmd_lock = threading.Lock()

        self._state_positions: list[float] | None = None
        self._state_pressures: list[list[float]] | None = None
        self._state_temperatures: list[list[float]] | None = None
        self._state_press_lost: list[int] | None = None
        self._state_ts: float = 0.0
        self._state_lock = threading.Lock()
        self._state_sub: Any | None = None
        if HandState_ is not None:
            self._state_sub = ChannelSubscriber(HAND_STATE_TOPIC_BY_SIDE[self.hand], HandState_)
            self._state_sub.Init(self._state_cb, 20)

    def _state_cb(self, msg: Any) -> None:
        positions: list[float] = []
        motor_state = list(getattr(msg, "motor_state", []) or [])
        press_sensor_state = list(getattr(msg, "press_sensor_state", []) or [])
        for idx in range(7):
            motor = motor_state[idx] if idx < len(motor_state) else None
            try:
                positions.append(float(getattr(motor, "q")))
            except Exception:
                return
        pressures: list[list[float]] = []
        temperatures: list[list[float]] = []
        lost_flags: list[int] = []
        for sensor in press_sensor_state:
            try:
                pressures.append([float(value)
                                 for value in list(getattr(sensor, "pressure", []) or [])])
                temperatures.append([float(value)
                                    for value in list(getattr(sensor, "temperature", []) or [])])
                lost_flags.append(int(getattr(sensor, "lost", 0)))
            except Exception:
                pressures = []
                temperatures = []
                lost_flags = []
                break
        if len(positions) == 7:
            with self._state_lock:
                self._state_positions = positions
                self._state_pressures = pressures or None
                self._state_temperatures = temperatures or None
                self._state_press_lost = lost_flags or None
                self._state_ts = time.time()

    def _read_actual_positions(self, max_age: float = 1.0) -> list[float] | None:
        with self._state_lock:
            if self._state_positions is None:
                return None
            if (time.time() - self._state_ts) > max_age:
                return None
            return clamp_hand_targets(self.hand, list(self._state_positions))

    def get_state_snapshot(self, max_age: float = 1.0) -> dict[str, Any] | None:
        with self._state_lock:
            if self._state_positions is None:
                return None
            age_s = time.time() - self._state_ts
            if age_s > max_age:
                return None
            return {
                "hand": self.hand,
                "positions": clamp_hand_targets(self.hand, list(self._state_positions)),
                "pressures": None if self._state_pressures is None else [list(row) for row in self._state_pressures],
                "temperatures": None if self._state_temperatures is None else [list(row) for row in self._state_temperatures],
                "lost": None if self._state_press_lost is None else list(self._state_press_lost),
                "timestamp": float(self._state_ts),
                "age_s": float(age_s),
            }

    def get_tactile_pressures(self, max_age: float = 1.0) -> list[list[float]] | None:
        snapshot = self.get_state_snapshot(max_age=max_age)
        if snapshot is None:
            return None
        pressures = snapshot.get("pressures")
        return None if pressures is None else [list(row) for row in pressures]

    def _get_start_targets(self) -> list[float]:
        """Return best estimate of current positions for ramp start."""
        actual = self._read_actual_positions(max_age=1.0)
        if actual is not None:
            return actual
        # On first-ever call, wait briefly for the state subscriber to receive data.
        if self._state_sub is not None and self._last_targets is None:
            deadline = time.time() + 0.3
            while time.time() < deadline:
                actual = self._read_actual_positions(max_age=2.0)
                if actual is not None:
                    return actual
                time.sleep(0.02)
        if self._last_targets is not None:
            return clamp_hand_targets(self.hand, list(self._last_targets))
        return clamp_hand_targets(self.hand, hand_open_targets(self.hand))

    @staticmethod
    def _interpolate_targets(
        start: list[float],
        stop: list[float],
        *,
        alpha: float,
    ) -> list[float]:
        blend = min(1.0, max(0.0, float(alpha)))
        return [s + (e - s) * blend for s, e in zip(start, stop)]

    def _publish_targets_for(
        self,
        targets: list[float],
        *,
        seconds: float,
        rate_hz: float,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        targets = clamp_hand_targets(self.hand, targets)
        self.publish_for(
            build_hand_msg(targets, kp=kp, kd=kd, tau=tau),
            seconds=seconds,
            rate_hz=rate_hz,
        )
        self._last_targets = list(targets)

    def _targets_reached(self, targets: list[float], *, tolerance: float = 0.08) -> bool:
        actual = self._read_actual_positions(max_age=0.5)
        if actual is None:
            return False
        return all(abs(float(goal) - float(current)) <= float(tolerance) for goal, current in zip(targets, actual))

    def write_targets_once(
        self,
        targets: list[float],
        *,
        kp: float = 0.5,
        kd: float = 0.1,
        tau: float = 0.0,
        timeout: int = 0,
        first_write_timeout_s: float | None = None,
    ) -> bool:
        targets = clamp_hand_targets(self.hand, targets)
        msg = build_hand_msg(targets, kp=kp, kd=kd, tau=tau, timeout=timeout)
        ok = self._pub.Write(msg, timeout=first_write_timeout_s)
        self._last_targets = [float(value) for value in targets]
        return ok is not False

    def publish_for(
        self,
        msg: HandCmd_,
        seconds: float,
        rate_hz: float = 50.0,
        *,
        first_write_timeout_s: float | None = None,
    ) -> bool:
        steps = max(1, int(max(0.01, float(seconds)) * max(1.0, float(rate_hz))))
        dt = 1.0 / max(1.0, float(rate_hz))
        matched = True
        for step_idx in range(steps):
            timeout = first_write_timeout_s if step_idx == 0 else None
            ok = self._pub.Write(msg, timeout=timeout)
            if step_idx == 0 and ok is False:
                matched = False
            time.sleep(dt)
        return matched

    def _stop_release_thread(self) -> None:
        if self._release_stop is not None:
            self._release_stop.set()
        if self._release_thread is not None and self._release_thread.is_alive():
            self._release_thread.join(timeout=1.0)
        self._release_stop = None
        self._release_thread = None

    def set_targets(
        self,
        targets: list[float],
        hold_s: float = 0.6,
        rate_hz: float = 50.0,
        kp: float = 1.2,
        kd: float = 0.05,
        tau: float = 0.05,
        ramp_s: float | None = None,
    ) -> None:
        self._stop_release_thread()
        if len(targets) != 7:
            raise ValueError("Hand targets must contain 7 joint values.")

        with self._cmd_lock:
            target_list = clamp_hand_targets(self.hand, [float(value) for value in targets])
            rate = max(1.0, float(rate_hz))
            total_hold_s = max(0.0, float(hold_s))
            ramp_duration_s = min(
                total_hold_s,
                max(1.0 / rate, 0.25 if ramp_s is None else float(ramp_s)),
            )

            start_targets = clamp_hand_targets(self.hand, self._get_start_targets())
            ramp_ran = False
            if any(abs(dst - src) > 1e-6 for src, dst in zip(start_targets, target_list)) and ramp_duration_s > 0.0:
                ramp_ran = True
                ramp_steps = max(2, int(round(ramp_duration_s * rate)))
                step_dt = ramp_duration_s / float(ramp_steps)
                for step_idx in range(1, ramp_steps + 1):
                    alpha = float(step_idx) / float(ramp_steps)
                    interp_targets = self._interpolate_targets(
                        start_targets, target_list, alpha=alpha)
                    self._publish_targets_for(
                        interp_targets,
                        seconds=step_dt,
                        rate_hz=rate,
                        kp=kp,
                        kd=kd,
                        tau=tau,
                    )

            remaining_hold_s = max(0.0, total_hold_s - (ramp_duration_s if ramp_ran else 0.0))
            if remaining_hold_s > 0.0 or self._last_targets is None:
                self._publish_targets_for(
                    target_list,
                    seconds=remaining_hold_s if remaining_hold_s > 0.0 else (1.0 / rate),
                    rate_hz=rate,
                    kp=kp,
                    kd=kd,
                    tau=tau,
                )
            settle_deadline = time.time() + max(0.25, total_hold_s)
            while time.time() < settle_deadline and not self._targets_reached(target_list):
                self._publish_targets_for(
                    target_list,
                    seconds=1.0 / rate,
                    rate_hz=rate,
                    kp=kp,
                    kd=kd,
                    tau=tau,
                )

    def _pose_targets(self, pose_targets: list[float]) -> list[float]:
        targets = list(pose_targets)
        if self._last_targets is not None:
            targets[0] = float(self._last_targets[0])
        return targets

    def open(self, hold_s: float = 0.6, rate_hz: float = 50.0, ramp_s: float | None = None) -> None:
        self.set_targets(
            hand_open_targets(self.hand),
            hold_s=hold_s,
            rate_hz=rate_hz,
            kp=1.5,
            kd=0.1,
            tau=0.03,
            ramp_s=ramp_s,
        )

    def close(self, hold_s: float = 0.6, rate_hz: float = 50.0, ramp_s: float | None = None) -> None:
        self.set_targets(
            hand_closed_targets(self.hand),
            hold_s=hold_s,
            rate_hz=rate_hz,
            kp=1.5,
            kd=0.1,
            tau=0.03,
            ramp_s=ramp_s,
        )

    def release_fingers(
        self,
        hold_s: float = 0.5,
        rate_hz: float = 50.0,
        *,
        persistent: bool = False,
    ) -> None:
        self._stop_release_thread()
        targets = list(
            self._last_targets) if self._last_targets is not None else hand_open_targets(self.hand)
        msg = build_hand_msg(targets, kp=0.0, kd=0.0, tau=0.0, timeout=1)
        if persistent:
            stop_event = threading.Event()

            def _loop() -> None:
                dt = 1.0 / max(1.0, float(rate_hz))
                while not stop_event.is_set():
                    self._pub.Write(msg)
                    time.sleep(dt)

            self._release_stop = stop_event
            self._release_thread = threading.Thread(
                target=_loop,
                name=f"dex3-{self.hand}-release",
                daemon=True,
            )
            self._release_thread.start()
        else:
            self.publish_for(
                msg,
                seconds=hold_s,
                rate_hz=rate_hz,
                first_write_timeout_s=1.0,
            )
        self._last_targets = None

    def stop_release_fingers(self) -> None:
        self._stop_release_thread()

    def move_finger(self, finger_name: str, hold_s: float = 1.0, settle_s: float = 0.6, rate_hz: float = 50.0) -> None:
        finger = str(finger_name).strip().lower()
        if finger not in FINGER_TO_IDXS:
            raise ValueError(f"Unknown finger '{finger_name}'.")
        targets = self._pose_targets(hand_open_targets(self.hand))
        closed_targets = self._pose_targets(hand_closed_targets(self.hand))
        for idx in FINGER_TO_IDXS[finger]:
            targets[idx] = closed_targets[idx]
        self.open(hold_s=settle_s, rate_hz=rate_hz)
        self.set_targets(targets, hold_s=hold_s, rate_hz=rate_hz, tau=0.12)
        self.open(hold_s=settle_s, rate_hz=rate_hz)


__all__ = [
    "Dex3HandController",
    "FINGER_TO_IDXS",
    "HAND_CLOSED",
    "HAND_CLOSED_LIMITS",
    "HAND_MAX_LIMITS",
    "HAND_MIN_LIMITS",
    "HAND_OPEN",
    "HAND_OPEN_LIMITS",
    "HAND_STATE_TOPIC_BY_SIDE",
    "HAND_THUMB_0_HOLD_TARGETS",
    "build_hand_msg",
    "hand_closed_targets",
    "hand_grip_targets",
    "hand_mid_targets",
    "hand_open_targets",
    "pack_ris_mode",
]
