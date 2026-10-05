"""Typed records used by the MeshGuard Sim demonstration."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite


def _finite_nonnegative(value: float, field: str) -> None:
    if not isfinite(value) or value < 0:
        raise ValueError(f"{field} must be a finite, non-negative number")


@dataclass(frozen=True, slots=True)
class Vec2:
    """Planar position or velocity in arbitrary simulation units."""

    x: float
    y: float

    def __post_init__(self) -> None:
        if not isfinite(self.x) or not isfinite(self.y):
            raise ValueError("vector coordinates must be finite")

    def distance_to(self, other: "Vec2") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def scale(self, factor: float) -> "Vec2":
        return Vec2(self.x * factor, self.y * factor)


class NodeRole(str, Enum):
    COMMAND = "command"
    RELAY = "relay"
    SCOUT = "scout"


@dataclass(frozen=True, slots=True)
class DroneNode:
    """A virtual network node; position is scenario data, not a flight order."""

    node_id: str
    role: NodeRole
    position: Vec2
    radio_range: float
    scan_range: float = 0.0
    online: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.role, NodeRole):
            object.__setattr__(self, "role", NodeRole(self.role))
        if not self.node_id.strip():
            raise ValueError("node_id cannot be empty")
        _finite_nonnegative(self.radio_range, "radio_range")
        _finite_nonnegative(self.scan_range, "scan_range")


@dataclass(frozen=True, slots=True)
class RescueBeacon:
    """An anonymous synthetic signal used only by the in-memory scenario."""

    beacon_id: str
    position: Vec2
    velocity: Vec2 = Vec2(0.0, 0.0)

    def __post_init__(self) -> None:
        if not self.beacon_id.strip():
            raise ValueError("beacon_id cannot be empty")


@dataclass(frozen=True, slots=True)
class Observation:
    beacon_id: str
    observer_id: str
    position: Vec2
    tick: int

    def __post_init__(self) -> None:
        if self.tick < 0:
            raise ValueError("tick must be non-negative")


@dataclass(frozen=True, slots=True)
class MeshPacket:
    packet_id: str
    source_id: str
    observation: Observation
    ttl: int
    path: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.ttl < 0:
            raise ValueError("ttl must be non-negative")


@dataclass(frozen=True, slots=True)
class Track:
    beacon_id: str
    position: Vec2
    velocity: Vec2
    last_seen_tick: int
    observer_count: int
    confidence: float

