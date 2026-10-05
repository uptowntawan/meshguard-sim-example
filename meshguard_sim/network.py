"""In-memory radio graph, parent assignment, and bounded packet relay."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .model import DroneNode, MeshPacket, NodeRole


@dataclass(frozen=True, slots=True)
class RelayResult:
    packet_id: str
    delivered_to: tuple[str, ...]
    paths: tuple[tuple[str, ...], ...]


class MeshNetwork:
    """A deterministic, in-memory mesh; it opens no sockets or radio devices."""

    def __init__(self, nodes: tuple[DroneNode, ...], command_id: str) -> None:
        self.nodes = {node.node_id: node for node in nodes}
        if len(self.nodes) != len(nodes):
            raise ValueError("node IDs must be unique")
        if command_id not in self.nodes:
            raise ValueError("command_id must identify a node")
        if self.nodes[command_id].role is not NodeRole.COMMAND:
            raise ValueError("command_id must identify the command node")
        self.command_id = command_id
        self.links = self._build_links()
        self.parents = self._assign_parents()

    def _build_links(self) -> dict[str, tuple[str, ...]]:
        links: dict[str, list[str]] = {node_id: [] for node_id in self.nodes}
        active = [node for node in self.nodes.values() if node.online]
        for index, left in enumerate(active):
            for right in active[index + 1 :]:
                # Both endpoints must be able to reach the other.
                distance = left.position.distance_to(right.position)
                if distance <= min(left.radio_range, right.radio_range):
                    links[left.node_id].append(right.node_id)
                    links[right.node_id].append(left.node_id)
        return {node_id: tuple(sorted(neighbors)) for node_id, neighbors in links.items()}

    def _assign_parents(self) -> dict[str, str | None]:
        """Choose a stable command-rooted spanning tree using shortest hop count."""
        parents: dict[str, str | None] = {self.command_id: None}
        queue = deque([self.command_id])
        while queue:
            parent = queue.popleft()
            for child in self.links[parent]:
                if child not in parents:
                    parents[child] = parent
                    queue.append(child)
        return {node_id: parents.get(node_id) for node_id in self.nodes}

    def route(self, source_id: str, destination_id: str) -> tuple[str, ...] | None:
        """Return a shortest path, including endpoints, or None if disconnected."""
        if source_id not in self.nodes or destination_id not in self.nodes:
            raise KeyError("route endpoint is not in this network")
        if not self.nodes[source_id].online or not self.nodes[destination_id].online:
            return None
        queue = deque([(source_id, (source_id,))])
        visited = {source_id}
        while queue:
            node_id, path = queue.popleft()
            if node_id == destination_id:
                return path
            for neighbor in self.links[node_id]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + (neighbor,)))
        return None

    def relay(self, packet: MeshPacket, max_hops: int = 8) -> RelayResult:
        """Fan a packet out over a bounded tree, once per reachable node."""
        if packet.source_id not in self.nodes:
            raise KeyError(f"unknown packet source: {packet.source_id}")
        if max_hops < 0:
            raise ValueError("max_hops must be non-negative")
        if not self.nodes[packet.source_id].online or packet.ttl == 0:
            return RelayResult(packet.packet_id, (), ())

        delivered: list[str] = []
        paths: list[tuple[str, ...]] = []
        queue = deque([(packet.source_id, (packet.source_id,), 0)])
        visited = {packet.source_id}
        while queue:
            node_id, path, hops = queue.popleft()
            if node_id != packet.source_id:
                delivered.append(node_id)
                paths.append(path)
            if hops >= min(max_hops, packet.ttl):
                continue
            for neighbor in self.links[node_id]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + (neighbor,), hops + 1))
        return RelayResult(packet.packet_id, tuple(delivered), tuple(paths))
