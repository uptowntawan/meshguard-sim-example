"""Deterministic virtual search-and-rescue exercise."""

from __future__ import annotations

from dataclasses import dataclass

from .model import DroneNode, MeshPacket, Observation, RescueBeacon, Track, Vec2
from .network import MeshNetwork
from .tracking import BeaconTracker


@dataclass(frozen=True, slots=True)
class TickReport:
    tick: int
    parent_links: tuple[tuple[str, str | None], ...]
    relayed_packets: int
    tracks: tuple[Track, ...]


class SearchAndRescueSimulation:
    """Run synthetic detections and mesh relays without external I/O."""

    def __init__(self, nodes: tuple[DroneNode, ...], beacons: tuple[RescueBeacon, ...]) -> None:
        self.nodes = nodes
        self.beacons = beacons
        self.network = MeshNetwork(nodes, command_id="mother")
        self.tracker = BeaconTracker()
        self.tick = 0
        self._packet_sequence = 0

    def advance(self) -> TickReport:
        """Generate detections, relay each report, and return the fused view."""
        relayed = 0
        for node in self.nodes:
            if not node.online or node.scan_range <= 0:
                continue
            for beacon in self.beacons:
                # Node positions stay fixed. Synthetic beacon velocity exercises
                # track estimation without controlling any node.
                position = beacon.position + beacon.velocity.scale(self.tick)
                if node.position.distance_to(position) > node.scan_range:
                    continue
                observation = Observation(beacon.beacon_id, node.node_id, position, self.tick)
                self._packet_sequence += 1
                packet = MeshPacket(
                    packet_id=f"p{self._packet_sequence:04d}",
                    source_id=node.node_id,
                    observation=observation,
                    ttl=8,
                )
                result = self.network.relay(packet, max_hops=8)
                relayed += len(result.delivered_to)
                # The command node's fused view updates only when the virtual
                # report reaches it (or when it was the source itself).
                if self.network.command_id in result.delivered_to:
                    self.tracker.ingest(observation)
                elif node.node_id == self.network.command_id:
                    self.tracker.ingest(observation)

        report = TickReport(
            tick=self.tick,
            parent_links=tuple(sorted(self.network.parents.items())),
            relayed_packets=relayed,
            tracks=self.tracker.snapshot(self.tick),
        )
        self.tick += 1
        return report


def demo_scenario() -> SearchAndRescueSimulation:
    """Create a fixed scenario with one command node, relays, and scouts."""
    nodes = (
        DroneNode("mother", role="command", position=Vec2(0, 0), radio_range=55),
        DroneNode("relay-east", role="relay", position=Vec2(45, 0), radio_range=55),
        DroneNode("relay-north", role="relay", position=Vec2(45, 45), radio_range=55),
        DroneNode("scout-1", role="scout", position=Vec2(90, 45), radio_range=55, scan_range=32),
        DroneNode("scout-2", role="scout", position=Vec2(0, 35), radio_range=55, scan_range=30),
    )
    # Synthetic identifiers and positions only; no people or live coordinates.
    beacons = (
        RescueBeacon("beacon-alpha", position=Vec2(84, 43), velocity=Vec2(-1, 0.5)),
        RescueBeacon("beacon-bravo", position=Vec2(8, 52), velocity=Vec2(0.2, -0.3)),
    )
    return SearchAndRescueSimulation(nodes, beacons)
