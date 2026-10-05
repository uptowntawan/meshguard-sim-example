"""Synthetic thermal-grid segmentation and screen-space reacquisition demo.

Frames are generated in memory. This module does not read cameras, control a
pan/tilt lens, or identify people. Intensities are unitless demo values, not
temperatures or calibrated sensor measurements.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite


@dataclass(frozen=True, slots=True)
class HeatFrame:
    """A row-major matrix of synthetic heat intensities in the range [0, 1]."""

    width: int
    height: int
    intensities: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.width < 1 or self.height < 1:
            raise ValueError("frame dimensions must be positive")
        if len(self.intensities) != self.width * self.height:
            raise ValueError("intensity count must equal width * height")
        if any(not isfinite(value) or not 0.0 <= value <= 1.0 for value in self.intensities):
            raise ValueError("intensities must be finite values between 0 and 1")

    def at(self, x: int, y: int) -> float:
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError("pixel coordinate is outside the frame")
        return self.intensities[y * self.width + x]


@dataclass(frozen=True, slots=True)
class HeatDetection:
    """One connected hot region, expressed as one-based row-major cell IDs."""

    cells: tuple[int, ...]
    center_x: float
    center_y: float
    peak: float

    @property
    def cell_span(self) -> str:
        if not self.cells:
            return ""
        if len(self.cells) == 1:
            return str(self.cells[0])
        return f"{min(self.cells)}-{max(self.cells)}"


class HeatGridSegmenter:
    """Threshold the synthetic intensity matrix and group touching cells."""

    def __init__(self, threshold: float = 0.72, min_cells: int = 1) -> None:
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("threshold must be between 0 and 1")
        if min_cells < 1:
            raise ValueError("min_cells must be at least one")
        self.threshold = threshold
        self.min_cells = min_cells

    def detect(self, frame: HeatFrame) -> tuple[HeatDetection, ...]:
        """Return hot regions using 8-neighbor connectivity."""
        hot = {
            (x, y)
            for y in range(frame.height)
            for x in range(frame.width)
            if frame.at(x, y) >= self.threshold
        }
        detections: list[HeatDetection] = []
        while hot:
            start = min(hot, key=lambda point: (point[1], point[0]))
            hot.remove(start)
            frontier = [start]
            component = [start]
            while frontier:
                x, y = frontier.pop()
                for ny in range(max(0, y - 1), min(frame.height, y + 2)):
                    for nx in range(max(0, x - 1), min(frame.width, x + 2)):
                        point = (nx, ny)
                        if point in hot:
                            hot.remove(point)
                            frontier.append(point)
                            component.append(point)
            if len(component) < self.min_cells:
                continue
            cells = tuple(sorted(y * frame.width + x + 1 for x, y in component))
            detections.append(
                HeatDetection(
                    cells=cells,
                    center_x=sum(x for x, _ in component) / len(component),
                    center_y=sum(y for _, y in component) / len(component),
                    peak=max(frame.at(x, y) for x, y in component),
                )
            )
        return tuple(detections)


class TrackState(str, Enum):
    OBSERVED = "observed"
    SEARCHING = "searching"
    REACQUIRED = "reacquired"
    LOST = "lost"


@dataclass(frozen=True, slots=True)
class GridTrack:
    track_id: str
    state: TrackState
    cells: tuple[int, ...]
    center_x: float
    center_y: float
    predicted_x: float
    predicted_y: float
    last_seen_tick: int


@dataclass(slots=True)
class _TrackMemory:
    track_id: str
    center_x: float
    center_y: float
    velocity_x: float
    velocity_y: float
    cells: tuple[int, ...]
    last_seen_tick: int
    misses: int = 0
    state: TrackState = TrackState.OBSERVED


class GridTracker:
    """Associate nearby anonymous heat regions across synthetic frames."""

    def __init__(self, max_gap_ticks: int = 3, max_match_cells: float = 4.0) -> None:
        if max_gap_ticks < 0:
            raise ValueError("max_gap_ticks must be non-negative")
        if not isfinite(max_match_cells) or max_match_cells <= 0:
            raise ValueError("max_match_cells must be finite and positive")
        self.max_gap_ticks = max_gap_ticks
        self.max_match_cells = max_match_cells
        self._tracks: dict[str, _TrackMemory] = {}
        self._next_id = 1
        self._last_tick = -1

    def update(self, tick: int, detections: tuple[HeatDetection, ...]) -> tuple[GridTrack, ...]:
        """Update tracks; short gaps enter SEARCHING and can return as REACQUIRED."""
        if tick <= self._last_tick:
            raise ValueError("tick values must increase")
        self._last_tick = tick

        candidates: list[tuple[float, str, int]] = []
        for track_id, track in self._tracks.items():
            if track.misses > self.max_gap_ticks:
                continue
            elapsed = max(1, tick - track.last_seen_tick)
            predicted_x = track.center_x + track.velocity_x * elapsed
            predicted_y = track.center_y + track.velocity_y * elapsed
            for index, detection in enumerate(detections):
                distance = ((predicted_x - detection.center_x) ** 2 + (predicted_y - detection.center_y) ** 2) ** 0.5
                if distance <= self.max_match_cells:
                    candidates.append((distance, track_id, index))

        matched_tracks: set[str] = set()
        matched_detections: set[int] = set()
        for _, track_id, index in sorted(candidates):
            if track_id in matched_tracks or index in matched_detections:
                continue
            track = self._tracks[track_id]
            detection = detections[index]
            elapsed = tick - track.last_seen_tick
            if elapsed > 0:
                track.velocity_x = (detection.center_x - track.center_x) / elapsed
                track.velocity_y = (detection.center_y - track.center_y) / elapsed
            track.center_x = detection.center_x
            track.center_y = detection.center_y
            track.cells = detection.cells
            track.state = TrackState.REACQUIRED if track.misses else TrackState.OBSERVED
            track.last_seen_tick = tick
            track.misses = 0
            matched_tracks.add(track_id)
            matched_detections.add(index)

        for track_id, track in self._tracks.items():
            if track_id not in matched_tracks:
                track.misses += 1
                track.state = (
                    TrackState.SEARCHING
                    if track.misses <= self.max_gap_ticks
                    else TrackState.LOST
                )

        for index, detection in enumerate(detections):
            if index in matched_detections:
                continue
            track_id = f"heat-{self._next_id:03d}"
            self._next_id += 1
            self._tracks[track_id] = _TrackMemory(
                track_id=track_id,
                center_x=detection.center_x,
                center_y=detection.center_y,
                velocity_x=0.0,
                velocity_y=0.0,
                cells=detection.cells,
                last_seen_tick=tick,
            )

        return self.snapshot(tick)

    def snapshot(self, tick: int) -> tuple[GridTrack, ...]:
        """Return current and recently missing tracks with screen-space predictions."""
        result: list[GridTrack] = []
        for track in sorted(self._tracks.values(), key=lambda item: item.track_id):
            elapsed = max(0, tick - track.last_seen_tick)
            result.append(
                GridTrack(
                    track_id=track.track_id,
                    state=track.state,
                    cells=track.cells,
                    center_x=track.center_x,
                    center_y=track.center_y,
                    predicted_x=track.center_x + track.velocity_x * elapsed,
                    predicted_y=track.center_y + track.velocity_y * elapsed,
                    last_seen_tick=track.last_seen_tick,
                )
            )
        return tuple(result)


def _synthetic_frame(width: int, height: int, hot_cell: int | None) -> HeatFrame:
    """Make a blank or single-cell synthetic frame; cell IDs start at one."""
    values = [0.05] * (width * height)
    if hot_cell is not None:
        if not 1 <= hot_cell <= width * height:
            raise ValueError("hot_cell is outside the synthetic grid")
        values[hot_cell - 1] = 0.96
    return HeatFrame(width, height, tuple(values))


def thermal_reacquisition_demo() -> tuple[tuple[int, tuple[GridTrack, ...]], ...]:
    """Demonstrate a synthetic heat source moving 20 -> 30 -> hidden -> 40."""
    segmenter = HeatGridSegmenter(threshold=0.72)
    tracker = GridTracker(max_gap_ticks=3, max_match_cells=4.0)
    frame_cells: tuple[int | None, ...] = (20, 30, None, 40, None, None, None, None)
    reports: list[tuple[int, tuple[GridTrack, ...]]] = []
    for tick, cell in enumerate(frame_cells):
        frame = _synthetic_frame(width=10, height=10, hot_cell=cell)
        reports.append((tick, tracker.update(tick, segmenter.detect(frame))))
    return tuple(reports)
