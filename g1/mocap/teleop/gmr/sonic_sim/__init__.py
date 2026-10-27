"""SONIC MuJoCo simulation environment ported from GR00T-WholeBodyControl.

Provides a standalone MuJoCo + DDS simulation that receives motor commands
from the C++ WBC deployment (gear_sonic_deploy) over CycloneDDS and publishes
simulated sensor state back, so the WBC policy sees the sim as a real robot.
"""

from sonic_sim.simulator_factory import SimulatorFactory, init_channel
from sonic_sim.configs import SimLoopConfig
from sonic_sim.base_sim import BaseSimulator, DefaultEnv

__all__ = [
    "SimulatorFactory",
    "init_channel",
    "SimLoopConfig",
    "BaseSimulator",
    "DefaultEnv",
]
