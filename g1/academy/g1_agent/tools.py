"""The agent's basic robot tools on sdk_wrapper.G1 (replaces
llm_client/robot_tools.build_robot_tools, which drives sdk_client.Robot).

Same names, arguments, clamping and result strings ("error: ..." marks a
failure).  Two intentional differences:
  - grab/release use sdk_wrapper's per-wrist Dex3 open/closed targets; the
    legacy tool used one fixed joint list for both hands, which closes the
    crossed Dex3 units in the wrong direction.
  - reach_forward runs the reviewed, table-aware extend recipe of the grab
    pipeline (grasp_pipeline.GraspPipeline.run_extend_arm_ee_ik) instead of
    sdk_client.Robot.extend_arm_forward()'s delta trajectory.
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable, Dict

from .robot_port import AgentHost


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def build_robot_tools(robot: Any, host: AgentHost, extend_arm: Callable[[str, float], Dict[str, Any]]) -> Dict[str, Callable[..., str]]:
    """``extend_arm(side, duration_s)`` performs the reach; see registry.py."""
    import sdk_wrapper

    def move(direction: str, distance_m: float = 0.5, yaw_rad: float = 0.0, speed_mps: float = 0.3) -> str:
        """Drive the base in a cardinal direction with optional yaw."""
        d = str(direction).strip().lower()
        if d == "stop":
            robot.loco_stop()
            return "stopped"
        if d not in {"forward", "backward", "left", "right"}:
            return f"error: unknown direction '{direction}'"
        try:
            speed = max(0.05, min(0.6, _finite_float(speed_mps, "speed_mps")))
            distance = max(0.0, min(5.0, _finite_float(distance_m, "distance_m")))
            yaw = _finite_float(yaw_rad, "yaw_rad")
        except (TypeError, ValueError) as exc:
            return f"error: {exc}"
        conflict = host.conflict()
        if conflict:
            return f"error: {conflict}"
        duration = distance / speed if speed > 0 else 0.0
        vx = speed if d == "forward" else -speed if d == "backward" else 0.0
        vy = speed if d == "left" else -speed if d == "right" else 0.0
        vyaw = yaw / duration if duration > 0.05 else 0.0
        robot.loco_move(vx, vy, vyaw, duration_s=duration)
        return f"moved {d} {distance:.2f} m at {speed:.2f} m/s with yaw {yaw:+.2f} rad over {duration:.2f} s"

    def reach_forward(height_m: float = 0.0, length_m: float = 0.4, arm: str = "right", duration_s: float = 3.0) -> str:
        """Reach the arm forward (height_m/length_m kept for schema compatibility)."""
        side = str(arm).strip().lower()
        if side not in {"left", "right"}:
            return f"error: arm must be 'left' or 'right', got '{arm}'"
        try:
            h = max(-0.30, min(0.30, _finite_float(height_m, "height_m")))
            reach = max(0.10, min(0.60, _finite_float(length_m, "length_m")))
            duration = _finite_float(duration_s, "duration_s")
        except (TypeError, ValueError) as exc:
            return f"error: {exc}"
        if duration <= 0 or duration > 30.0:
            return "error: duration_s must be > 0 and <= 30 seconds"
        conflict = host.conflict()
        if conflict:
            return f"error: {conflict}"
        result = extend_arm(side, duration)
        if not result.get("success"):
            return f"error: reach {side} arm failed: {result.get('reason')}"
        return (f"reached {side} arm with the extend recipe (stages {', '.join(result.get('stages', []))}; "
                f"requested height={h:+.2f} m, length={reach:.2f} m are not used)")

    def grab(hand: str = "right", resistance_threshold: float = 0.5, n_steps: int = 8, step_s: float = 0.15) -> str:
        """Close the fingers stepwise; stop when finger torque exceeds the threshold (N·m)."""
        side = str(hand).strip().lower()
        if side not in {"left", "right"}:
            return f"error: hand must be 'left' or 'right', got '{hand}'"
        try:
            n = max(1, int(n_steps))
            dt = max(0.05, _finite_float(step_s, "step_s"))
            thresh = max(0.0, _finite_float(resistance_threshold, "resistance_threshold"))
        except (TypeError, ValueError) as exc:
            return f"error: {exc}"
        robot.open_dex3_hand(hand=side, hold_s=0.2)
        time.sleep(0.2)
        opened, closed = sdk_wrapper.HAND_SAFE_OPEN[side], sdk_wrapper.HAND_CLOSED[side]
        for step in range(1, n + 1):
            alpha = step / n
            robot.hand_pose([(1.0 - alpha) * o + alpha * c for o, c in zip(opened, closed)], hand=side, hold_s=dt)
            time.sleep(0.05)
            snap = robot.get_dex3_hand_sensors(side)
            torques = [abs(t) for t in (snap.get("torques") or {}).values() if t is not None] if isinstance(snap, dict) else []
            tau_max = max(torques) if torques else 0.0
            if tau_max >= thresh:
                return (f"resistance hit at {int(alpha * 100)}% closure "
                        f"(tau_max={tau_max:.2f} N·m, threshold={thresh:.2f}); held.")
        return "fully closed without exceeding resistance threshold"

    def release(hand: str = "right") -> str:
        """Open the fingers fully."""
        side = str(hand).strip().lower()
        if side not in {"left", "right"}:
            return f"error: hand must be 'left' or 'right', got '{hand}'"
        robot.open_dex3_hand(hand=side, hold_s=0.4)
        return f"opened {side} hand"

    return {"move": move, "reach_forward": reach_forward, "grab": grab, "release": release}
