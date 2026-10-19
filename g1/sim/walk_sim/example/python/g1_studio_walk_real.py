#!/usr/bin/env python3
"""Conservative raw-``rt/lowcmd`` stepping console for a real Unitree G1.

This is deliberately *not* a replacement for Unitree's balance controller.
It sends all 29 body joints through ``rt/lowcmd`` and therefore takes control
away from ai_sport.  The arrow-key motions are small, open-loop test steps;
use them only with a safety harness, clear space, an attentive spotter, and a
robot already in a stable standing pose.  They are unsuitable for untethered
walking or recovery from a disturbance.

Controls (the terminal must have focus):
  d          toggle sdk_wrapper.G1.toggle_dev_mode() and seed the hold pose
  Up/Down    queue one small forward/backward left/right test-step pair
  Left/Right queue one small lateral left/right weight-shift test
  Space      cancel the motion and hold the *measured* pose immediately
  r          re-seed the hold pose from rt/lowstate (only while idle)
  q          hold, leave Dev Mode, and exit (Esc is reserved for arrow keys)

Example:
  python3 g1_studio_walk_real.py --iface eth0

The program will not move on an arrow key without ``--allow-motion``.  That
extra acknowledgement is intentional for raw leg control.
"""

import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path


# Let this example find the sibling academy package when launched from any cwd.
G1_ROOT = Path(__file__).resolve().parents[4]
if str(G1_ROOT / "academy") not in sys.path:
    sys.path.insert(0, str(G1_ROOT / "academy"))

from sdk_wrapper import G1, LOWCMD_JOINTS  # noqa: E402


HZ = 50.0
LEG_KP = [60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40]
LEG_KD = [1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 1]
DEFAULT_KP = LEG_KP + [60, 40, 40] + [40] * 14
DEFAULT_KD = LEG_KD + [1, 1, 1] + [1] * 14
# g1_studio_walk.py reads scene.xml qpos0.  All 29 actuator positions there
# are zero.  This is a *simulation kinematic initializer*, not a calibrated
# physical G1 stand; it must never be the default lowcmd takeover target.
SIM_STAND_Q = [0.0] * 29
JOINT_NAMES = (
    "L hip-pitch", "L hip-roll", "L hip-yaw", "L knee", "L ankle-pitch", "L ankle-roll",
    "R hip-pitch", "R hip-roll", "R hip-yaw", "R knee", "R ankle-pitch", "R ankle-roll",
)
# Mechanical limits for the 12 leg joints, matching the academy's validated
# G1 joint table.  Every generated phase is clamped before it reaches DDS.
LEG_LIMITS = (
    (-2.5307, 2.8798), (-0.5236, 2.9671), (-2.7576, 2.7576), (-0.0873, 2.8798), (-0.8727, 0.5236), (-0.2618, 0.2618),
    (-2.5307, 2.8798), (-2.9671, 0.5236), (-2.7576, 2.7576), (-0.0873, 2.8798), (-0.8727, 0.5236), (-0.2618, 0.2618),
)


def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class RawStepConsole:
    """Owns a continuous all-body lowcmd stream and a small action queue."""

    def __init__(self, robot, args):
        self.robot, self.args = robot, args
        self.dev_mode = False
        self.q_cmd = None
        self.q_goal = None
        self.mode_machine = 0
        self.action = []
        self.phase_started = None
        self.status = "Waiting for rt/lowstate; press d to enter Dev Mode."
        self.last_display = 0.0

    def sensed(self, timeout=2.0):
        """Require a current state rather than commanding an old pose."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.robot.get_lowstate()
            if state and len(state["joint_positions"]) >= len(LOWCMD_JOINTS):
                return state
            time.sleep(0.02)
        raise TimeoutError("No rt/lowstate received; refusing to command lowcmd.")

    def seed_from_state(self):
        state = self.sensed()
        self.q_cmd = list(state["joint_positions"][:29])
        self.q_goal = list(self.q_cmd)
        raw = state.get("raw")
        self.mode_machine = int(getattr(raw, "mode_machine", 0))
        return state

    def set_dev_mode(self):
        if self.dev_mode:
            self.status = "Dev Mode is already on."
            return
        self.seed_from_state()  # Never take ownership before seeing fresh q.
        result = self.robot.toggle_dev_mode()
        if isinstance(result, dict) and result.get("code", 0) != 0:
            raise RuntimeError("toggle_dev_mode failed: %s" % result)
        self.dev_mode = True
        if self.args.initial_pose == "sim" and self.args.allow_motion:
            self.action = [(list(SIM_STAND_Q), self.args.stand_ramp_s,
                            "ramp to MuJoCo qpos0 stand")]
            self.phase_started = None
            self.status = "DEV MODE ON: DANGEROUS experimental ramp to MuJoCo qpos0 stand."
        else:
            self.status = "DEV MODE ON: holding measured full-body pose on rt/lowcmd."

    def leave_dev_mode(self):
        if not self.dev_mode:
            return
        self.abort(measured=True)
        self.publish()  # Send the measured hold frame once before giving ownership back.
        result = self.robot.toggle_dev_mode()
        if isinstance(result, dict) and result.get("code", 0) != 0:
            raise RuntimeError("toggle_dev_mode failed: %s" % result)
        self.dev_mode = False
        self.status = "Dev Mode off; ai_sport ownership restored."

    def abort(self, measured=False):
        if measured:
            try:
                self.seed_from_state()
            except TimeoutError:
                pass
        self.action = []
        self.phase_started = None
        if self.q_cmd is not None:
            self.q_goal = list(self.q_cmd)
        self.status = "Motion cancelled; holding pose."

    def _phase(self, offsets, duration, label):
        target = list(self.q_cmd)
        for jid, delta in offsets.items():
            lo, hi = LEG_LIMITS[jid]
            target[jid] = clamp(target[jid] + delta, lo, hi)
        return (target, duration, label)

    def queue_step(self, forward):
        """Queue a tiny two-foot open-loop test step relative to the hold pose.

        Values are intentionally small.  The sequence shifts, lifts one foot,
        sets it down, then repeats for the other foot; it is not a gait engine.
        """
        if not self._can_move():
            return
        s = 1.0 if forward else -1.0
        # hip-roll signs are hardware conventions: mirrored shifts below move
        # the pelvis over the intended stance foot on the standard G1 setup.
        phases = [
            ({1: -0.055, 7: -0.055}, 0.70, "shift onto right foot"),
            ({0: -s * 0.055, 3: 0.16, 4: -s * 0.045, 1: -0.055, 7: -0.055}, 0.55, "left foot lift"),
            ({0: -s * 0.080, 3: 0.04, 4: -s * 0.030, 1: -0.055, 7: -0.055}, 0.55, "left foot place"),
            ({1: 0.055, 7: 0.055}, 0.70, "shift onto left foot"),
            ({6: -s * 0.055, 9: 0.16, 10: -s * 0.045, 1: 0.055, 7: 0.055}, 0.55, "right foot lift"),
            ({6: -s * 0.080, 9: 0.04, 10: -s * 0.030, 1: 0.055, 7: 0.055}, 0.55, "right foot place"),
            ({}, 0.80, "return to neutral hold"),
        ]
        base = list(self.q_cmd)
        self.action = []
        for offsets, duration, label in phases:
            target = list(base)
            for jid, delta in offsets.items():
                lo, hi = LEG_LIMITS[jid]
                target[jid] = clamp(target[jid] + delta, lo, hi)
            self.action.append((target, duration, label))
        self.phase_started = None
        self.status = "Queued %s test step." % ("forward" if forward else "backward")

    def queue_shift(self, left):
        if not self._can_move():
            return
        sign = 1.0 if left else -1.0
        base = list(self.q_cmd)
        self.action = []
        for offsets, duration, label in (
            ({1: sign * 0.060, 7: sign * 0.060}, 0.8, "lateral weight shift"),
            ({}, 0.8, "return to neutral hold"),
        ):
            target = list(base)
            for jid, delta in offsets.items():
                lo, hi = LEG_LIMITS[jid]
                target[jid] = clamp(target[jid] + delta, lo, hi)
            self.action.append((target, duration, label))
        self.phase_started = None
        self.status = "Queued %s weight shift." % ("left" if left else "right")

    def _can_move(self):
        if not self.dev_mode:
            self.status = "Press d first: Dev Mode is required for rt/lowcmd legs."
            return False
        if not self.args.allow_motion:
            self.status = "Restart with --allow-motion to acknowledge raw leg movement."
            return False
        return True

    def update_action(self, now):
        if not self.action:
            return
        target, duration, label = self.action[0]
        if self.phase_started is None:
            self.phase_started = now
            self.phase_from = list(self.q_goal)
            self.status = "Step phase: %s" % label
        alpha = smoothstep((now - self.phase_started) / duration)
        self.q_goal = [a + (b - a) * alpha for a, b in zip(self.phase_from, target)]
        if alpha >= 1.0:
            self.action.pop(0)
            self.phase_started = None

    def publish(self):
        if not self.dev_mode or self.q_cmd is None:
            return
        # A velocity limiter applies even if a phase or key handler is wrong.
        max_delta = self.args.max_joint_speed / HZ
        self.q_cmd = [old + clamp(goal - old, -max_delta, max_delta)
                      for old, goal in zip(self.q_cmd, self.q_goal)]
        self.robot._lowcmd_client().write(
            self.q_cmd, self.mode_machine, kp=DEFAULT_KP, kd=DEFAULT_KD, dq=0.0, tau=0.0)

    def display(self):
        now = time.monotonic()
        if now - self.last_display < 0.15:
            return
        self.last_display = now
        try:
            state = self.robot.get_lowstate()
            if state is None:
                text = "no rt/lowstate"
            else:
                q, dq = state["joint_positions"], state["joint_velocities"]
                leg = " | ".join("%s %+.2f (%+.2f)" % (JOINT_NAMES[i], q[i], dq[i]) for i in range(12))
                imu = state.get("imu") or {}
                rpy = imu.get("rpy", [math.nan] * 3)
                text = "rpy(deg) %+.1f %+.1f %+.1f\n%s" % tuple([x * 180.0 / math.pi for x in rpy] + [leg])
        except Exception as exc:
            text = "state error: %s" % exc
        sys.stdout.write("\x1b[2J\x1b[H" +
                         "G1 RAW LOWCMD TEST CONSOLE  |  Dev Mode: %s  |  rate: %.0f Hz\n" %
                         ("ON" if self.dev_mode else "OFF", HZ) +
                         "d dev-mode  arrows test motion  space hold  r reseed  q quit\n" +
                         "STATUS: %s\n\n%s\n" % (self.status, text))
        sys.stdout.flush()


def read_key():
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    value = sys.stdin.read(1)
    if value == "\x1b" and select.select([sys.stdin], [], [], 0.050)[0]:
        value += sys.stdin.read(2)
    return value


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=__doc__)
    parser.add_argument("--iface", default="eth0", help="DDS network interface (default: eth0)")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain ID (default: 0)")
    parser.add_argument("--allow-motion", action="store_true", help="acknowledge that arrows may move raw leg joints")
    parser.add_argument("--max-joint-speed", type=float, default=0.25,
                        help="hard q-target slew limit in rad/s (default: 0.25)")
    parser.add_argument("--initial-pose", choices=("measured", "sim"), default="measured",
                        help="on Dev Mode entry, hold measured q (default); 'sim' is experimental and not a physical stand")
    parser.add_argument("--stand-ramp-s", type=float, default=5.0,
                        help="minimum smooth ramp time to the MuJoCo stand (default: 5 s)")
    args = parser.parse_args()
    if args.max_joint_speed <= 0.0 or args.max_joint_speed > 0.6:
        parser.error("--max-joint-speed must be > 0 and <= 0.6 rad/s")
    if args.stand_ramp_s < 2.0:
        parser.error("--stand-ramp-s must be at least 2 seconds")
    if not sys.stdin.isatty():
        parser.error("requires an interactive TTY for arrow-key control")

    console = RawStepConsole(G1(args.iface, domain_id=args.domain_id), args)
    old_tty = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    deadline = time.monotonic()
    leave_before_exit = False
    try:
        while True:
            key = read_key()
            if key in ("q", "Q"):
                leave_before_exit = True
                break
            try:
                if key in ("d", "D"):
                    if console.dev_mode:
                        console.leave_dev_mode()
                    else:
                        console.set_dev_mode()
                elif key == "\x1b[A":
                    console.queue_step(True)
                elif key == "\x1b[B":
                    console.queue_step(False)
                elif key == "\x1b[D":
                    console.queue_shift(True)
                elif key == "\x1b[C":
                    console.queue_shift(False)
                elif key == " ":
                    console.abort(measured=True)
                elif key in ("r", "R") and not console.action:
                    console.seed_from_state()
                    console.status = "Re-seeded hold pose from rt/lowstate."
            except Exception as exc:
                console.abort(measured=False)
                console.status = "INPUT ERROR; holding last safe target: %s" % exc

            now = time.monotonic()
            if now >= deadline:
                try:
                    console.update_action(now)
                    console.publish()
                except Exception as exc:
                    # Do not stop the hold stream or toggle ownership because a
                    # key/action handler had a transient error.
                    console.abort(measured=False)
                    console.status = "COMMAND ERROR; holding last safe target: %s" % exc
                console.display()
                deadline += 1.0 / HZ
                if deadline < now - 1.0 / HZ:
                    deadline = now + 1.0 / HZ
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        if leave_before_exit:
            try:
                console.leave_dev_mode()
            except Exception as exc:
                print("\nWARNING: could not leave Dev Mode: %s" % exc, file=sys.stderr)
        elif console.dev_mode:
            # Never retry a service *toggle* from finally.  An RPC timeout can
            # mean the first request reached the robot; retrying could reverse it.
            for _ in range(10):
                try:
                    console.publish()
                except Exception:
                    break
                time.sleep(1.0 / HZ)
            print("\nDev Mode left ON deliberately; verify ai_sport before changing ownership.", file=sys.stderr)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_tty)


if __name__ == "__main__":
    main()
