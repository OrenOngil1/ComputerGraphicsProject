# 2D–3D Pose Estimation on DEM Terrain

An interactive 3D graphics application in **C++ / OpenGL / OpenCV** that
simulates and solves the **Perspective-n-Point (PnP)** problem: recovering a
camera's 3D pose (position + orientation) from a 2D image. The world is a 3D
terrain built from a **Digital Elevation Map** — a greyscale image whose pixel
brightness is height.

The screen is split into two viewports: a **global** view (the whole terrain
from outside) and the **player** view (what the camera sees). Every
pose-estimation mode compares the camera's *true* pose against the *computed*
one, both as fly-through paths in the global view and as a translucent "ghost"
terrain in the player view.

## The core idea

> Given only what a camera sees, where is the camera?

Answering it needs **2D–3D correspondences**: pairs of (a known 3D world point,
the 2D pixel where it appears). With **four or more**, OpenCV's `solvePnP`
recovers the camera's six degrees of freedom. The four modes are four ways of
*producing* those correspondences — from fully manual to fully automatic.

## The four modes

| Mode | Key | How correspondences are made | Demonstrates |
| ---- | --- | ---------------------------- | ------------ |
| **A — Navigation** | — | none (fly, record path `R`, play back `Ctrl+R`) | Dual-view free-flight camera + path recording |
| **B — Picking** | `P` | *manual* — click points on the terrain | Human-in-the-loop PnP (the baseline) |
| **C — Trackers** | `T` | *fiducial* — uniquely coloured spheres with known 3D centres; find each colour's blob, its centroid is the 2D point | Automatic correspondence with trivial data association |
| **D — Feature Matching** | `F` | *natural* — ORB features on the terrain itself, matched against a pre-built database | The hard, markerless case |

Modes C and D share a base (`PoseComparisonState`) that records
`(true pose, computed pose)` per timestep (`B` to capture, `N`/`M` to review)
and draws the comparison identically.

Full controls and a per-mode walkthrough: **[docs/pose-estimation-modes.md](docs/pose-estimation-modes.md)**.

## Build & run

Linux (the project targets WSL2 Ubuntu 22.04). Dependencies:

```
g++ make pkg-config libopencv-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libglm-dev
```

```bash
make            # build  -> bin/drone_sim
./bin/drone_sim # run; pick a terrain by number at the prompt
make check      # run the headless math checks (no GPU needed)
```

> The Makefile does not track header dependencies — after editing a header or
> pulling, run `make clean && make`.

## How it works (key techniques)

- **Render-to-read-back.** To learn where things land on screen, the app
  renders an off-screen pass and reads the pixels: an **ID pass** colours each
  terrain vertex by its index (so a pixel reveals the 3D point under it), and a
  **tracker pass** draws the spheres flat on black. Depth testing stays on, so
  occluded geometry correctly disappears from the read-back.
- **Mode C (blobs).** Classify each read-back pixel against the known sphere
  palette; each colour's mean pixel is its centroid (the 2D point), paired with
  the sphere's known 3D centre.
- **Mode D (ORB + RANSAC).** Pre-phase renders each recorded waypoint, detects
  ORB keypoints, and anchors each to 3D via the ID pass → a `FeatureDb`.
  Run-phase detects features in the live view, matches them (Hamming + Lowe's
  ratio test), and solves with **RANSAC PnP** for robustness to wrong matches.
- **Lighting.** A directional light (Lambert + ambient) with per-vertex normals
  from the height map; `L` cycles presets. Required for the Mode D experiment.

Architecture (composition root, State pattern, Renderer/vision separation):
**[docs/architecture-notes.md](docs/architecture-notes.md)**.

## The lighting experiment

Changing the light between building the feature database and using it (PDF
p. 54) **breaks Mode D almost completely**: under the database's own light,
well-overlapping views localise to sub-percent error; any change of light
*direction* collapses matches from 30–303 down to 2–14. The cause is that the
terrain's appearance is **shading-driven, not texture-driven** — moving the sun
flips the very intensity gradients ORB encodes. Full results and analysis:
**[docs/lighting-experiment.md](docs/lighting-experiment.md)**.

## Tests

`make check` builds `tests/headless_checks.cpp` against the vision + loader code
(no GPU) and verifies, on synthetic inputs with known answers: terrain normals,
tracker blob centroids, and both PnP solvers (including RANSAC outlier rejection
and the minimum-inlier guard). The PnP checks reproject points with the
GL-correct square-pixel camera on a non-square viewport, so they catch focal-
length / aspect mistakes in the intrinsics.

## Repository layout

```
src/core/      composition root, scene state, camera, lighting
src/state/     one State per mode (Navigation, Pick, Trackers, FeatureMatch)
src/render/    Renderer — all GPU work and read-back captures
src/vision/    OpenCV: PnP solvers, blob detection, ORB feature matching
src/loader/    DEM image -> terrain mesh + normals
src/engine/    vendored BasicOpenGL toolkit (do not modify)
assets/        shaders + terrain DEMs
tests/         headless math checks (make check)
docs/          architecture notes, mode guide, experiment write-up
```

## Documentation index

- [docs/pose-estimation-modes.md](docs/pose-estimation-modes.md) — modes C/D, full keyboard reference, experiment procedure
- [docs/lighting-experiment.md](docs/lighting-experiment.md) — the lighting experiment: method, results, analysis
- [docs/architecture-notes.md](docs/architecture-notes.md) — module map, ownership, the State pattern
