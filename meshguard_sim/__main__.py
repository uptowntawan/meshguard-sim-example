"""Command-line entry point for the offline MeshGuard Sim demo."""

from __future__ import annotations

import argparse

from .simulation import demo_scenario
from .thermal_grid import thermal_reacquisition_demo
from .radial_guide import virtual_recenter_hint


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="meshguard-sim",
        description="Offline mesh relay and anonymous rescue-beacon simulation.",
    )
    parser.add_argument("--ticks", type=int, default=3, help="number of simulation steps (1-100)")
    parser.add_argument(
        "--thermal-demo",
        action="store_true",
        help="show a synthetic thermal grid and temporary reacquisition example",
    )
    args = parser.parse_args()
    if not 1 <= args.ticks <= 100:
        parser.error("--ticks must be between 1 and 100")

    if args.thermal_demo:
        print("Synthetic heat grid | 10x10 | cell IDs are row-major from 1 | no camera I/O")
        for tick, tracks in thermal_reacquisition_demo():
            if not tracks:
                print(f"TICK {tick}: no heat regions")
                continue
            for track in tracks:
                cells = ",".join(str(cell) for cell in track.cells) or "none"
                guide = virtual_recenter_hint(
                    track.predicted_x,
                    track.predicted_y,
                    width=10,
                    height=10,
                    goal_ring=1,
                    ring_count=20,
                    max_shift_cells=1.0,
                )
                print(
                    f"TICK {tick}: {track.track_id} {track.state.value} "
                    f"cells={cells} predicted=({track.predicted_x:.1f},{track.predicted_y:.1f}) "
                    f"ring={guide.ring}/20 virtual_shift=({guide.shift_x_cells:+.1f},"
                    f"{guide.shift_y_cells:+.1f}) within_ring_1={guide.within_goal} "
                    f"last_seen={track.last_seen_tick}"
                )
        return 0

    simulation = demo_scenario()
    print("MeshGuard Sim | offline virtual scenario | no hardware I/O")
    for _ in range(args.ticks):
        report = simulation.advance()
        print(f"\nTICK {report.tick} | relayed deliveries: {report.relayed_packets}")
        for node_id, parent_id in report.parent_links:
            parent = parent_id if parent_id is not None else "ROOT"
            print(f"  topology {parent} -> {node_id}")
        if not report.tracks:
            print("  no virtual beacon observations")
        for track in report.tracks:
            print(
                f"  {track.beacon_id}: pos=({track.position.x:.1f}, {track.position.y:.1f}) "
                f"vel=({track.velocity.x:.2f}, {track.velocity.y:.2f}) "
                f"observers={track.observer_count} confidence={track.confidence:.2f}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
