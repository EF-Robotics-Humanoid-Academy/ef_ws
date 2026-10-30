"""Arm, hand and walking commands for the chat, on the /sort V2 pipeline.

The chat runs on the dashboard's one SorterV2 (perception, IK loop, validator,
state machine), so /sort shows the state of a chat command and its Stop button
cancels it.  Commands go to a single FIFO worker: one physical command at a
time, in the order asked for.  A command that fails or is stopped clears what
was queued behind it.  Long commands report their result through ``notify``;
the chat stays usable meanwhile, which is what makes a spoken "stop" work.
"""
from __future__ import annotations

import json
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

DIRECTIONS = {  # base frame: x forward, y left, z up
    "forward": (1.0, 0.0, 0.0), "backward": (-1.0, 0.0, 0.0), "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0), "up": (0.0, 0.0, 1.0), "down": (0.0, 0.0, -1.0),
}
DEFAULT_HAND_STEP_M = 0.05
MAX_HAND_STEP_M = 0.15
# Without a table in view the clearance cannot be checked: small steps only.
MAX_UNCHECKED_STEP_M = 0.05
MAX_QUEUED = 8
WAIT_QUICK_S = 25.0
STABLE_HOLD_POSE = "stable hold"
STABLE_HOLD_FILE = Path(__file__).resolve().parents[2] / "WBC" / "saved_ik_pose_cli_v3_poses.json"
# sdk_wrapper.LEFT_ARM_JOINTS / RIGHT_ARM_JOINTS; not imported so this module stays robot-independent.
ARM_JOINTS = {"left": range(15, 22), "right": range(22, 29)}


def load_stable_hold(path: Path = STABLE_HOLD_FILE, name: str = STABLE_HOLD_POSE) -> Dict[str, List[float]]:
    """Both arms of the WBC tool's saved balanced hold pose, right arm first."""
    poses = json.loads(Path(path).read_text(encoding="utf-8")).get("poses", [])
    for pose in poses:
        if str(pose.get("name", "")).strip().lower() == name:
            joints = pose["arm_joints"]
            return {side: [float(joints[str(j)]) for j in ARM_JOINTS[side]] for side in ("right", "left")}
    raise RuntimeError(f"pose {name!r} was not found in {path}")


@dataclass
class _Job:
    label: str
    run: Callable[[], Tuple[bool, str]]
    done: threading.Event = field(default_factory=threading.Event)
    ok: bool = False
    message: str = ""
    reported: bool = False


class Manipulator:
    """``sorter()`` returns the shared SorterV2, building it on first use, and
    may raise when there is none (offline); ``built_sorter()`` returns it only
    if it already exists; ``conflict()`` names another controller owning the
    robot; ``notify(text)`` reports a result the chat turn has already moved past."""

    def __init__(self, sorter: Callable[[], Any], built_sorter: Callable[[], Any], conflict: Callable[[], Optional[str]],
                 notify: Callable[[str], None]) -> None:
        self._sorter = sorter
        self._built_sorter = built_sorter
        self._conflict = conflict
        self._notify = notify
        self._jobs: "queue.Queue[_Job]" = queue.Queue()
        self._lock = threading.Lock()
        self._current: Optional[_Job] = None
        self._stopping = False
        self._thread = threading.Thread(target=self._worker, name="g1-chat-motion", daemon=True)
        self._thread.start()

    # -- worker ---------------------------------------------------------------
    def _worker(self) -> None:
        while True:
            job = self._jobs.get()
            with self._lock:
                self._current = job
                self._stopping = False
            try:
                conflict = self._conflict()
                job.ok, job.message = (False, conflict) if conflict else job.run()
            except Exception as exc:
                job.ok, job.message = False, f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._current = None
                job.done.set()
                late = job.reported
            if late:
                self._notify(job.message if job.ok else f"{job.label} failed: {job.message}")
            if not job.ok:
                self._clear_queue(f"not started because {job.label} failed")

    def _clear_queue(self, why: str) -> None:
        promised = []  # the operator was told these would report
        while True:
            try:
                job = self._jobs.get_nowait()
            except queue.Empty:
                break
            with self._lock:
                job.ok, job.message = False, why
                job.done.set()
                if job.reported:
                    promised.append(job)
        for job in promised:
            self._notify(f"{job.label}: {why}")

    def busy(self) -> bool:
        with self._lock:
            return self._current is not None or not self._jobs.empty()

    def submit(self, label: str, run: Callable[[], Tuple[bool, str]], wait_s: float = 0.0) -> Tuple[bool, str]:
        """Queue a command.  With ``wait_s`` the real result comes back when it
        finishes in time; otherwise "started" and the result goes to notify."""
        conflict = self._conflict()
        if conflict and not self.busy():
            return False, conflict
        if self._jobs.qsize() >= MAX_QUEUED:
            return False, "too many commands are queued already"
        job = _Job(label, run)
        queued_behind = self.busy()
        self._jobs.put(job)
        if wait_s > 0.0 and job.done.wait(wait_s):
            return job.ok, job.message
        with self._lock:
            if job.done.is_set():
                return job.ok, job.message
            job.reported = True
        return True, f"{label}: {'queued' if queued_behind else 'started'}; the result is reported when it finishes"

    def stop(self) -> str:
        """Software cancellation of the running and queued commands; no e-stop."""
        self._clear_queue("cancelled by a stop command")
        sorter = self._built_sorter()
        if sorter is not None:
            sorter.action("stop")
        return "stopped"

    # -- commands -------------------------------------------------------------
    def _sort_cmd(self, label: str, body: Callable[[Any], Tuple[bool, str]]) -> Callable[[], Tuple[bool, str]]:
        def run() -> Tuple[bool, str]:
            sorter = self._sorter()
            return sorter.chat_run(label, lambda: body(sorter))
        return run

    def _object_index(self, name: str) -> Tuple[Optional[int], str]:
        with_sorter = self._sorter()
        with with_sorter.lock:
            names = [o.name for o in with_sorter.objects]
        words = set(re.findall(r"[a-zäöüß0-9]+", str(name).lower())) - {"the", "a", "an", "my", "this", "that"}
        best = [i for i, n in enumerate(names) if words and words & set(n.lower().split())]
        if len(best) == 1:
            return best[0], names[best[0]]
        known = " and the ".join(names)
        if best:
            return None, f"'{name}' could be several objects: the {known}"
        return None, f"I only know how to grab the {known}; other objects can be set up in /sort"

    def _spot_index(self, spot: str) -> Tuple[Optional[int], str]:
        sorter = self._sorter()
        with sorter.lock:
            names = [p.name for p in sorter.patterns]
        match = re.search(r"\b([a-z])\s*$", str(spot).strip().lower())
        letter = match.group(1) if match else str(spot).strip().lower()
        for i, n in enumerate(names):
            if n.lower() == str(spot).strip().lower() or n.lower().split()[-1] == letter:
                return i, n
        return None, f"I only know {' and '.join(names)}"

    def grab(self, name: str) -> Tuple[bool, str]:
        index, resolved = self._object_index(name)
        if index is None:
            return False, resolved
        return self.submit(f"grabbing the {resolved}", self._sort_cmd(f"grab {resolved}", lambda s: s.chat_grab(index)))

    def place(self, spot: str) -> Tuple[bool, str]:
        index, resolved = self._spot_index(spot)
        if index is None:
            return False, resolved
        return self.submit(f"placing on {resolved}", self._sort_cmd(f"place on {resolved}", lambda s: s.chat_place(index)))

    def move_hand(self, side: str, direction: str, distance_m: float = DEFAULT_HAND_STEP_M) -> Tuple[bool, str]:
        side, direction = str(side).strip().lower(), str(direction).strip().lower()
        if side not in ("left", "right"):
            return False, "hand must be left or right"
        if direction not in DIRECTIONS:
            return False, f"direction must be one of {', '.join(DIRECTIONS)}"
        try:
            distance = float(distance_m)
        except (TypeError, ValueError):
            return False, "distance must be a number"
        distance = max(0.01, min(MAX_HAND_STEP_M, distance))
        delta = [v * distance for v in DIRECTIONS[direction]]
        body = lambda sorter: sorter.chat_move_hand(side, delta, direction, MAX_UNCHECKED_STEP_M)  # noqa: E731
        return self.submit(f"moving the {side} hand {direction}", self._sort_cmd(f"{side} hand {direction}", body), WAIT_QUICK_S)

    def hand(self, side: str, action: str) -> Tuple[bool, str]:
        side, action = str(side).strip().lower(), str(action).strip().lower()
        if side not in ("left", "right") or action not in ("open", "close"):
            return False, "hand must be left or right, and the action open or close"
        return self.submit(f"{action} the {side} hand", self._sort_cmd(f"{action} {side} hand", lambda s: s.chat_hand(side, action)),
                           WAIT_QUICK_S)

    def hold_pose(self) -> Tuple[bool, str]:
        import numpy as np

        targets = {side: np.asarray(q, dtype=np.float64) for side, q in load_stable_hold().items()}
        return self.submit("going to the balanced hold pose",
                           self._sort_cmd("balanced hold pose", lambda s: s.chat_hold_pose(targets, "balanced hold")))

    def release_arms(self) -> Tuple[bool, str]:
        return self.submit("releasing the arms", self._sort_cmd("release arms", lambda s: s.chat_release_arms()), WAIT_QUICK_S)

    def background(self, label: str, run: Callable[[], Tuple[bool, str]], wait_s: float = 0.0) -> Tuple[bool, str]:
        """A non-arm motion (walking, navigation) in the same one-at-a-time queue."""
        return self.submit(label, run, wait_s)

    def free_arms_then(self, label: str, run: Callable[[], Tuple[bool, str]]) -> Tuple[bool, str]:
        """A high-level gesture: the arm action service only moves arms that no
        rt/arm_sdk command holds, so a holding controller is released first."""
        def job() -> Tuple[bool, str]:
            sorter = self._built_sorter()
            if sorter is not None:
                with sorter.p.lock:
                    if sorter.p.holding_object:
                        return False, "my right hand is holding something; place it first"
                if sorter.p._authority_engaged:
                    ok, message = sorter.chat_run("release arms for a gesture", sorter.chat_release_arms)
                    if not ok:
                        return False, message
            return run()
        return self.submit(label, job, WAIT_QUICK_S)

    # -- what the robot knows -------------------------------------------------
    def holding(self) -> Optional[str]:
        """Name of the object in the right hand, if the sorter is built and has one."""
        sorter = self._built_sorter()
        if sorter is None:
            return None
        with sorter.p.lock:
            if not sorter.p.holding_object:
                return None
        plan = sorter.current
        if plan is None:
            return "an object"
        with sorter.lock:
            return sorter.objects[plan.target.obj_index].name

    def scene(self, settle_s: float = 6.0) -> str:
        """Objects and spots in view; starts perception when it was idle."""
        sorter = self._sorter()
        sorter.touch()
        deadline = time.time() + settle_s
        text = sorter.chat_scene()
        while "none of the configured" in text and time.time() < deadline:
            time.sleep(0.4)
            sorter.touch()
            text = sorter.chat_scene()
        return text
