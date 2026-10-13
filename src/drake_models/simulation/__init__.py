"""Forward-dynamics simulation of exercise models under trajectory tracking.

``TrackingSimulator`` rolls a discrete Drake plant forward while its joint
actuators track a ``JointTrajectory`` (implicit PD plus support-consistent
gravity compensation) and returns the state and tracking-error time series.
"""

from drake_models.simulation.controller import (
    DEFAULT_DAMPING_RATIO,
    DEFAULT_STIFFNESS_MARGIN,
    JointTrajectory,
    pd_gains,
    stance_natural_frequency,
    support_gravity_compensation,
)
from drake_models.simulation.simulator import (
    DEFAULT_SAMPLE_PERIOD,
    DEFAULT_TIME_STEP,
    SUPPORT_BODIES,
    SimulationResult,
    TrackingSimulator,
)

__all__ = [
    "DEFAULT_DAMPING_RATIO",
    "DEFAULT_SAMPLE_PERIOD",
    "DEFAULT_STIFFNESS_MARGIN",
    "DEFAULT_TIME_STEP",
    "SUPPORT_BODIES",
    "JointTrajectory",
    "SimulationResult",
    "TrackingSimulator",
    "pd_gains",
    "stance_natural_frequency",
    "support_gravity_compensation",
]
