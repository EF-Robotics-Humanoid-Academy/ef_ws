"""What the agent backend needs from its host, and what the agent framework
needs from "the robot".

AgentHost: hooks the hosting application binds once it exists (the agent is
built before the dashboard).  arm_conflict() returns a reason while another
controller owns the robot (the dashboard's /control, /wbc or /sort) so motion
skills refuse instead of fighting it; grasp_pipeline() returns the shared
grasp_pipeline.GraspPipeline, so chat grabs and /sort use one arm lock and
one pose hold.  Unbound, a host allows motion and builds a private pipeline.

AgentRobotPort: G1Agent hands its ``robot`` to ExpressiveMotionController,
ActivityManager (headlight indicator) and NavigationAdapter, which probe it
with hasattr()/getattr() for a few sdk_client.Robot methods.  The port offers
exactly those hooks, implemented on sdk_wrapper.G1.  Skills do not use the
port; they call G1 directly (registry.py).
"""
from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

from .sensors import SensorWatch


class MotionBlocked(RuntimeError):
    pass


class AgentHost:
    def __init__(self, robot: Any, rgbd_endpoint: str = "tcp://127.0.0.1:5555") -> None:
        self.robot = robot
        self.rgbd_endpoint = rgbd_endpoint
        self._arm_conflict: Optional[Callable[[], Optional[str]]] = None
        self._pipeline_factory: Optional[Callable[[], Any]] = None
        self._own_pipeline: Any = None
        self._lock = threading.Lock()

    def bind(self, *, arm_conflict: Optional[Callable[[], Optional[str]]] = None,
             grasp_pipeline: Optional[Callable[[], Any]] = None) -> None:
        if arm_conflict is not None:
            self._arm_conflict = arm_conflict
        if grasp_pipeline is not None:
            self._pipeline_factory = grasp_pipeline

    def conflict(self) -> Optional[str]:
        return None if self._arm_conflict is None else self._arm_conflict()

    def require(self, what: str) -> None:
        conflict = self.conflict()
        if conflict:
            raise MotionBlocked(f"{what} not started: {conflict}")

    def grasp_pipeline(self) -> Any:
        if self._pipeline_factory is not None:
            return self._pipeline_factory()
        with self._lock:
            if self._own_pipeline is None:
                import grasp_pipeline

                self._own_pipeline = grasp_pipeline.GraspPipeline(self.robot, self.rgbd_endpoint)
            return self._own_pipeline


class AgentRobotPort:
    def __init__(self, robot: Any, sensors: SensorWatch, host: AgentHost) -> None:
        self.g1 = robot
        self.sensors = sensors
        self.host = host

    # agent.activity.HeadlightStateManager
    def headlight(self, color: Any = "white", intensity: int = 100, duration: Optional[float] = None) -> int:
        import sdk_wrapper

        if isinstance(color, (list, tuple)):
            color = tuple(int(v) for v in color)
        code = int(self.g1.set_headlight(color=color, intensity=intensity, duration_s=0 if duration is None else duration) or 0)
        # The LED RPC can time out (3104) although the LED changed; sdk_wrapper
        # treats that as accepted, so the indicator must too.
        return 0 if sdk_wrapper._led_control_was_accepted(code) else code

    # agent.navigation.NavigationAdapter
    def sensors_stale(self, max_age: float = 1.0) -> Dict[str, bool]:
        return self.sensors.stale(max_age)

    @property
    def _sensor_timestamps(self) -> Dict[str, float]:
        return self.sensors.timestamps()

    # agent.expressive_motion.ExpressiveMotionController
    def repeat(self, *, motion_file: str, **_kwargs: Any) -> Dict[str, Any]:
        self.host.require("expressive motion")
        return self.g1.repeat_motion_file(motion_file, arm="both")
