# MeshGuard Sim

MeshGuard Sim is a self-contained Python demonstration of a parent/child drone
network for a **simulated search-and-rescue beacon exercise**. It models node
availability, radio links, multi-hop message relay, and anonymous beacon tracks.

This project is simulation software only. It does not connect to flight
controllers, radios, cameras, GPS devices, or any other real hardware. It does
not issue movement or payload commands. The only tracked entities are virtual
beacons created inside the demo scenario; there is no person identification,
camera processing, or real-world target tracking.

## Run

Requires Python 3.10 or newer. From this directory, run:

```text
python -m meshguard_sim --ticks 5
```

The scenario is deterministic and uses only the Python standard library. Each
tick prints the virtual network's parent/child links, relayed beacon updates,
and the fused anonymous track table. Nothing is transmitted outside the
process.

## Architecture

- `model.py` contains validated, immutable simulation records.
- `network.py` builds radio reachability and relays expiring packets over
  multi-hop routes.
- `tracking.py` fuses simulated beacon observations and estimates planar
  velocity.
- `thermal_grid.py` thresholds a synthetic intensity grid, labels connected
  hot cells, and demonstrates short-gap track reacquisition in screen space.
- `radial_guide.py` classifies a point into concentric rings (0 at frame center)
  and calculates a bounded virtual image shift toward ring 1.
- `simulation.py` runs a deterministic search-and-rescue scenario.
- `__main__.py` provides the command-line entry point.

This is a teaching/demo baseline, not an aviation safety system or a deployable
drone fleet controller.

To see the 10x10 synthetic heat-grid example, run:

```text
python -m meshguard_sim --thermal-demo
```

Cell IDs count left-to-right, top-to-bottom, starting at 1. The demo uses
unitless generated intensities; it does not ingest camera frames, perform
person identification, or move a camera lens. A displayed thermal color palette
is not a calibrated temperature measurement.

The radial guide divides the radius from the frame center to its farthest
corner into 20 bands. Ring 0 is the exact center; ring 1 is the innermost band.
Its `virtual_shift` output is measured in grid cells and demonstrates image
reframing only. It is not a pan/tilt, yaw/pitch, servo, or drone movement command.
