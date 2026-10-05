"""Concentric screen-space zones and a virtual recentering hint."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot, isfinite


@dataclass(frozen=True, slots=True)
class RadialPosition:
    """A point's distance from frame center, classified into ring 0..N."""

    ring: int
    ring_count: int
    distance_cells: float
    max_radius_cells: float


@dataclass(frozen=True, slots=True)
class VirtualRecenterHint:
    """Suggested image-coordinate shift; this is not a camera actuator command."""

    ring: int
    goal_ring: int
    shift_x_cells: float
    shift_y_cells: float
    within_goal: bool


def radial_position(
    x: float,
    y: float,
    width: int,
    height: int,
    ring_count: int = 20,
) -> RadialPosition:
    """Classify a screen point into concentric rings around the frame center.

    Ring 0 contains the exact center. Rings 1..ring_count divide the distance
    from center to the farthest frame corner into equal radial bands.
    """
    if width < 1 or height < 1:
        raise ValueError("frame dimensions must be positive")
    if ring_count < 1:
        raise ValueError("ring_count must be positive")
    if not isfinite(x) or not isfinite(y):
        raise ValueError("screen coordinates must be finite")

    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0
    max_radius = hypot(center_x, center_y)
    distance = hypot(x - center_x, y - center_y)
    if distance == 0 or max_radius == 0:
        ring = 0
    else:
        ring = min(ring_count, max(1, ceil((distance / max_radius) * ring_count)))
    return RadialPosition(ring, ring_count, distance, max_radius)


def virtual_recenter_hint(
    x: float,
    y: float,
    width: int,
    height: int,
    *,
    goal_ring: int = 1,
    ring_count: int = 20,
    max_shift_cells: float = 3.0,
) -> VirtualRecenterHint:
    """Suggest a bounded shift of the virtual image toward the goal ring.

    The vector is in image-grid coordinates, not yaw/pitch, servo, or motor
    units. Integrating this with a physical lens or vehicle is intentionally
    outside this simulator.
    """
    if not 0 <= goal_ring <= ring_count:
        raise ValueError("goal_ring must be between 0 and ring_count")
    if not isfinite(max_shift_cells) or max_shift_cells < 0:
        raise ValueError("max_shift_cells must be finite and non-negative")

    position = radial_position(x, y, width, height, ring_count)
    if position.ring <= goal_ring or position.distance_cells == 0:
        return VirtualRecenterHint(position.ring, goal_ring, 0.0, 0.0, True)

    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0
    unit_x = (center_x - x) / position.distance_cells
    unit_y = (center_y - y) / position.distance_cells
    goal_radius = position.max_radius_cells * goal_ring / ring_count
    desired_shift = position.distance_cells - goal_radius
    step = min(desired_shift, max_shift_cells)
    return VirtualRecenterHint(
        ring=position.ring,
        goal_ring=goal_ring,
        shift_x_cells=unit_x * step,
        shift_y_cells=unit_y * step,
        within_goal=False,
    )
