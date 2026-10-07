"""Reserved handoff after Nav2 arrival. No motion/manipulation is implemented."""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ClosedApproachRequest:
    target_id: str
    target_map_point: Any
    navigation_goal: Any


def request_closed_approach(request: ClosedApproachRequest) -> str:
    """TODO: stationary re-observation -> closed approach -> wrist manipulation.

    Use fresh sensor observations here: the frozen Nav2 point is only a coarse
    destination. Add quality checks, explicit completion/failure and then a
    wrist-camera handoff. This placeholder sends no velocity or arm commands.
    """
    return 'CLOSED_APPROACH_PENDING'
