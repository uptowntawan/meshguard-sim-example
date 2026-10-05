"""Fusion of anonymous synthetic beacon detections."""

from __future__ import annotations

from collections import defaultdict

from .model import Observation, Track, Vec2


class BeaconTracker:
    """Keep a short in-memory history and estimate planar beacon motion."""

    def __init__(self, retention_ticks: int = 12) -> None:
        if retention_ticks < 1:
            raise ValueError("retention_ticks must be at least one")
        self.retention_ticks = retention_ticks
        self._observations: dict[str, dict[int, list[Observation]]] = defaultdict(
            lambda: defaultdict(list)
        )

    def ingest(self, observation: Observation) -> bool:
        """Add an observation; duplicate observer/beacon/tick reports are ignored."""
        bucket = self._observations[observation.beacon_id][observation.tick]
        if any(item.observer_id == observation.observer_id for item in bucket):
            return False
        bucket.append(observation)
        self._prune(observation.tick)
        return True

    def _prune(self, now: int) -> None:
        oldest = now - self.retention_ticks + 1
        for beacon_id in list(self._observations):
            history = self._observations[beacon_id]
            for tick in list(history):
                if tick < oldest:
                    del history[tick]
            if not history:
                del self._observations[beacon_id]

    @staticmethod
    def _mean(points: list[Vec2]) -> Vec2:
        return Vec2(
            sum(point.x for point in points) / len(points),
            sum(point.y for point in points) / len(points),
        )

    def snapshot(self, now: int) -> tuple[Track, ...]:
        self._prune(now)
        tracks: list[Track] = []
        for beacon_id, history in sorted(self._observations.items()):
            latest_tick = max(history)
            latest = history[latest_tick]
            position = self._mean([item.position for item in latest])
            prior_ticks = [tick for tick in history if tick < latest_tick]
            velocity = Vec2(0.0, 0.0)
            if prior_ticks:
                prior_tick = max(prior_ticks)
                prior = self._mean([item.position for item in history[prior_tick]])
                elapsed = latest_tick - prior_tick
                velocity = (position - prior).scale(1.0 / elapsed)
            age = max(0, now - latest_tick)
            observers = len({item.observer_id for item in latest})
            confidence = max(0.0, min(1.0, (0.45 + 0.15 * observers) * (0.9**age)))
            tracks.append(
                Track(
                    beacon_id=beacon_id,
                    position=position,
                    velocity=velocity,
                    last_seen_tick=latest_tick,
                    observer_count=observers,
                    confidence=confidence,
                )
            )
        return tuple(tracks)
