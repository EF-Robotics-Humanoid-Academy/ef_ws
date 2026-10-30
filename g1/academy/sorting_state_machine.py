"""Explicit state machine and structured event log for the V2 /sort pipeline.

The sorter (sorting_v2.SorterV2) drives the robot; this module only records
where it is and refuses transitions the sequence does not allow, so a failure
can never silently advance it.  Pure Python: unit-testable without a robot.
"""
from __future__ import annotations

import enum
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, FrozenSet, List, Optional


class State(str, enum.Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    DETECTING = "DETECTING"
    VALIDATING_TARGET = "VALIDATING_TARGET"
    MOVING_TO_PREGRASP = "MOVING_TO_PREGRASP"
    APPROACHING = "APPROACHING"
    CLOSING_HAND = "CLOSING_HAND"
    VERIFYING_GRASP = "VERIFYING_GRASP"
    LIFTING = "LIFTING"
    VERIFYING_LIFT = "VERIFYING_LIFT"
    HOLDING_OBJECT = "HOLDING_OBJECT"   # chat grab finished: waiting for a place command
    MOVING_TO_DROPOFF = "MOVING_TO_DROPOFF"
    LOWERING = "LOWERING"
    RELEASING_OBJECT = "RELEASING_OBJECT"
    VERIFYING_PLACEMENT = "VERIFYING_PLACEMENT"
    RETRACTING = "RETRACTING"
    READY_FOR_NEXT_OBJECT = "READY_FOR_NEXT_OBJECT"
    FINALIZING = "FINALIZING"
    COMPLETED = "COMPLETED"
    RECOVERING = "RECOVERING"
    ABORTED = "ABORTED"
    ERROR = "ERROR"


class Failure(str, enum.Enum):
    """Why a phase did not succeed; each maps to its own recovery."""
    NO_TARGET = "no_target"                # nothing (consistent) to pick
    UNREACHABLE = "unreachable"            # object or drop spot outside the arm's reach
    IK_FAILURE = "ik_failure"              # no level-palm IK solution along the path
    COLLISION_REJECTED = "collision_rejected"  # validator kept rejecting the next step
    NOT_REACHED = "not_reached"            # moving but the EE never got within tolerance
    TRACKING_FAILURE = "tracking_failure"  # measured joints cannot follow the reference
    FINGER_FAILURE = "finger_failure"      # fingers jammed / could not be commanded
    GRASP_NOT_CONFIRMED = "grasp_not_confirmed"
    LIFT_INCOMPLETE = "lift_incomplete"
    OBJECT_LOST = "object_lost"
    PLACEMENT_FAILURE = "placement_failure"
    RETRACTION_FAILURE = "retraction_failure"
    PREPARATION_FAILURE = "preparation_failure"
    CONTROLLER_CONFLICT = "controller_conflict"
    SENSOR_UNAVAILABLE = "sensor_unavailable"
    OPERATOR_STOP = "operator_stop"
    INTERNAL = "internal"


@dataclass
class PhaseResult:
    ok: bool
    failure: Optional[Failure] = None
    detail: str = ""
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def success(cls, detail: str = "", **data: Any) -> "PhaseResult":
        return cls(True, None, detail, dict(data))

    @classmethod
    def fail(cls, failure: Failure, detail: str = "", **data: Any) -> "PhaseResult":
        return cls(False, failure, detail, dict(data))


S = State
_ANY_ACTIVE = frozenset(s for s in State if s not in (S.IDLE, S.COMPLETED, S.ABORTED, S.ERROR))
# Allowed forward transitions; RECOVERING / ABORTED / ERROR are reachable from
# every active state and are handled separately.
TRANSITIONS: Dict[State, FrozenSet[State]] = {
    S.IDLE: frozenset({S.PREPARING}),
    S.PREPARING: frozenset({S.DETECTING}),
    S.DETECTING: frozenset({S.VALIDATING_TARGET, S.FINALIZING}),
    S.VALIDATING_TARGET: frozenset({S.MOVING_TO_PREGRASP, S.DETECTING}),
    S.MOVING_TO_PREGRASP: frozenset({S.APPROACHING}),
    S.APPROACHING: frozenset({S.CLOSING_HAND}),
    S.CLOSING_HAND: frozenset({S.VERIFYING_GRASP}),
    S.VERIFYING_GRASP: frozenset({S.LIFTING}),
    S.LIFTING: frozenset({S.VERIFYING_LIFT}),
    S.VERIFYING_LIFT: frozenset({S.MOVING_TO_DROPOFF, S.HOLDING_OBJECT}),
    S.HOLDING_OBJECT: frozenset({S.MOVING_TO_DROPOFF}),
    S.MOVING_TO_DROPOFF: frozenset({S.LOWERING}),
    S.LOWERING: frozenset({S.RELEASING_OBJECT}),
    S.RELEASING_OBJECT: frozenset({S.VERIFYING_PLACEMENT}),
    S.VERIFYING_PLACEMENT: frozenset({S.RETRACTING}),
    S.RETRACTING: frozenset({S.READY_FOR_NEXT_OBJECT}),
    S.READY_FOR_NEXT_OBJECT: frozenset({S.DETECTING, S.FINALIZING}),
    S.RECOVERING: frozenset({S.DETECTING, S.READY_FOR_NEXT_OBJECT, S.FINALIZING}),
    S.FINALIZING: frozenset({S.COMPLETED}),
    S.COMPLETED: frozenset({S.PREPARING}),
    S.ABORTED: frozenset({S.PREPARING}),
    S.ERROR: frozenset({S.PREPARING}),
}
# Human-readable headline per state for the dashboard.
HEADLINES = {
    S.IDLE: "IDLE", S.PREPARING: "PREPARING — moving to the preparation pose", S.DETECTING: "DETECTING",
    S.VALIDATING_TARGET: "VALIDATING TARGET", S.MOVING_TO_PREGRASP: "MOVING TO PRE-GRASP", S.APPROACHING: "APPROACHING",
    S.CLOSING_HAND: "GRASPING — closing fingers", S.VERIFYING_GRASP: "VERIFYING GRASP", S.LIFTING: "LIFTING",
    S.VERIFYING_LIFT: "VERIFYING LIFT", S.HOLDING_OBJECT: "HOLDING OBJECT", S.MOVING_TO_DROPOFF: "MOVING TO DROP-OFF", S.LOWERING: "LOWERING",
    S.RELEASING_OBJECT: "RELEASING OBJECT", S.VERIFYING_PLACEMENT: "VERIFYING PLACEMENT", S.RETRACTING: "RETRACTING",
    S.READY_FOR_NEXT_OBJECT: "READY FOR NEXT OBJECT", S.FINALIZING: "FINALIZING", S.COMPLETED: "SORT COMPLETE",
    S.RECOVERING: "RECOVERING", S.ABORTED: "ABORTED", S.ERROR: "ERROR — operator intervention required",
}


class TransitionError(RuntimeError):
    pass


@dataclass
class Event:
    ts: float
    state: str
    kind: str          # "transition" | "info" | "warn" | "error"
    message: str
    data: Dict[str, Any] = field(default_factory=dict)
    repeats: int = 1

    def as_dict(self) -> Dict[str, Any]:
        return {"ts": self.ts, "time": time.strftime("%H:%M:%S", time.localtime(self.ts)), "state": self.state,
                "kind": self.kind, "message": self.message, "data": self.data, "repeats": self.repeats}


class EventLog:
    """Bounded structured log; an event identical to the previous one only
    bumps its repeat counter (a blocked approach logs one line, not hundreds)."""

    def __init__(self, maxlen: int = 400) -> None:
        self._events: Deque[Event] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, state: State, kind: str, message: str, **data: Any) -> Event:
        with self._lock:
            last = self._events[-1] if self._events else None
            if last is not None and last.kind == kind and last.message == message and last.state == state.value:
                last.repeats += 1
                last.ts = time.time()
                last.data = dict(data)
                return last
            event = Event(time.time(), state.value, kind, message, dict(data))
            self._events.append(event)
            return event

    def tail(self, n: int = 200) -> List[Dict[str, Any]]:
        with self._lock:
            return [e.as_dict() for e in list(self._events)[-n:]]

    def lines(self, n: int = 200) -> List[str]:
        out = []
        for e in self.tail(n):
            rep = f" (x{e['repeats']})" if e["repeats"] > 1 else ""
            out.append(f"{e['time']} [{e['state']}] {e['kind']}: {e['message']}{rep}")
        return out

    def clear(self) -> None:
        with self._lock:
            self._events.clear()


class SortStateMachine:
    """Current state, guarded transitions, failure bookkeeping."""

    def __init__(self, log: Optional[EventLog] = None) -> None:
        self.state = State.IDLE
        self.entered_at = time.time()
        self.log = log or EventLog()
        self.last_failure: Optional[Failure] = None
        self.last_failure_detail = ""
        self.recovery_reason = ""
        self.history: Deque[Dict[str, Any]] = deque(maxlen=100)
        self._lock = threading.RLock()

    def can_enter(self, target: State) -> bool:
        if target in (S.RECOVERING, S.ABORTED, S.ERROR):
            return self.state in _ANY_ACTIVE or target == S.ERROR
        return target in TRANSITIONS.get(self.state, frozenset())

    def enter(self, target: State, message: str = "", **data: Any) -> None:
        with self._lock:
            if not self.can_enter(target):
                raise TransitionError(f"illegal transition {self.state.value} -> {target.value}")
            previous = self.state
            self.state = target
            self.entered_at = time.time()
            self.history.append({"ts": self.entered_at, "from": previous.value, "to": target.value, "message": message})
            self.log.add(target, "transition", f"{previous.value} -> {target.value}" + (f": {message}" if message else ""),
                         **data)

    def fail(self, result: PhaseResult) -> None:
        with self._lock:
            self.last_failure = result.failure
            self.last_failure_detail = result.detail
            self.log.add(self.state, "error", f"{result.failure.value if result.failure else 'failure'}: {result.detail}",
                         **result.data)

    def info(self, message: str, **data: Any) -> None:
        self.log.add(self.state, "info", message, **data)

    def warn(self, message: str, **data: Any) -> None:
        self.log.add(self.state, "warn", message, **data)

    def reset(self) -> None:
        with self._lock:
            self.state = State.IDLE
            self.entered_at = time.time()
            self.last_failure = None
            self.last_failure_detail = ""
            self.recovery_reason = ""

    def time_in_state(self) -> float:
        return time.time() - self.entered_at

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {"state": self.state.value, "headline": HEADLINES[self.state], "since_s": self.time_in_state(),
                    "last_failure": self.last_failure.value if self.last_failure else None,
                    "last_failure_detail": self.last_failure_detail, "recovery_reason": self.recovery_reason,
                    "history": list(self.history)[-20:]}
