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
| **B — Picking** | `P` | *manual* — click a 2D point in the camera view, then its 3D match on the map | Human-in-the-loop PnP (the baseline) |
| **C — Trackers** | `T` | *fiducial* — uniquely coloured spheres with known 3D centres; find each colour's blob, its centroid is the 2D point | Automatic correspondence with trivial data association |
| **D — Feature Matching** | `F` | *ORB-assisted manual* — ORB suggests salient points one at a time; the user hand-places each on the map to build the database; the run-phase then matches against it | The markerless case, anchored by hand |

Modes C and D share a base (`PoseComparisonState`) that records
`(true pose, computed pose)` per timestep (`B` to capture, `N`/`M` to review)
and draws the comparison identically.

Full controls and a per-mode walkthrough: **[docs/pose-estimation-modes.md](docs/pose-estimation-modes.md)**.

## Build & run

The project builds with **CMake** on Linux, Windows, and macOS.

### Dependencies

A setup script per platform installs the prerequisites:

| OS | Script | Installs |
| -- | ------ | -------- |
| Linux (apt) | `scripts/setup.sh` | `cmake g++ libglfw3-dev libopencv-dev` |
| macOS (brew) | `scripts/setup.sh` | `cmake glfw opencv` |
| Windows | `scripts/setup.ps1` | checks for the MSVC compiler (offers a winget install) and bootstraps **vcpkg** |

On Windows the libraries are then fetched automatically by vcpkg the first time
you configure (manifest mode via `vcpkg.json`, versions pinned by its
`builtin-baseline`). On Linux/macOS the system packages above supply them.

### CMake

Generator + toolchain choices live in `CMakePresets.json` (presets `linux` and
`windows`), so configuring is a single command:

```bash
# Linux / macOS
cmake --preset linux
cmake --build build
ctest --test-dir build            # headless math checks
./build/bin/drone_sim

# Windows — from a "Developer PowerShell for VS 2022"
cmake --preset windows
cmake --build build
ctest --test-dir build            # headless math checks
./build/bin/drone_sim.exe
```

> The `windows` preset reads the vcpkg toolchain from `$env{VCPKG_ROOT}`. A VS
> Developer environment provides that variable automatically; outside one, set
> it to your vcpkg checkout (e.g. `C:\vcpkg`).

### VS Code (CMake Tools)

Install the **CMake Tools** extension, pick the **windows** (or **linux**)
configure preset from the status bar, then use its **Configure / Build / Debug /
Run Tests** actions — no terminal needed. `.vscode/` is pre-wired: CMake Tools
sources the MSVC environment, the debugger runs the binary from the project root
(so `assets/` resolves), and IntelliSense follows the live CMake build.

## How it works (key techniques)

- **Render-to-read-back.** To learn where things land on screen, the app
  renders an off-screen pass and reads the pixels: an **ID pass** colours each
  terrain vertex by its index (so a pixel reveals the 3D point under it), and a
  **tracker pass** draws the spheres flat on black. Depth testing stays on, so
  occluded geometry correctly disappears from the read-back.
- **Mode C (blobs).** Classify each read-back pixel against the known sphere
  palette; each colour's mean pixel is its centroid (the 2D point), paired with
  the sphere's known 3D centre.
- **Mode D (ORB + RANSAC), manual anchoring.** Pre-phase is by hand: for each
  recorded view ORB suggests its strongest N points one at a time and the user
  color-picks each one's 3D spot on the global map → a `FeatureDb` of
  hand-placed `(descriptor, 3D)` pairs. Run-phase detects features in the live
  view, matches them against that database (Hamming + Lowe's ratio test), and
  solves with **RANSAC PnP** for robustness to wrong matches.
- **Lighting.** A directional light (Lambert + ambient) with per-vertex normals
  from the height map; `L` cycles presets. Required for the Mode D experiment.

Architecture (composition root, State pattern, Renderer/vision separation):
**[docs/architecture-notes.md](docs/architecture-notes.md)**.

## The lighting experiment

Because the terrain's appearance is **shading-driven, not texture-driven**,
moving the sun flips the very intensity gradients ORB encodes. With the manual
Mode D this shows up in the *detection* step: a light-direction change relocates
which points ORB suggests for the same view. The earlier automatic-matching
version made the same cause measurable as a near-total collapse (matches
30–303 → 2–14, pose refused), recorded in full at
**[docs/lighting-experiment.md](docs/lighting-experiment.md)**.

## Tests

`ctest --test-dir build` (or CMake Tools' **Run Tests**) builds and runs
`tests/headless_checks.cpp` against the vision + loader code (no GPU) and
verifies, on synthetic inputs with known answers: terrain normals,
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
tests/         headless math checks (ctest)
scripts/       per-OS dependency setup (setup.sh, setup.ps1)
docs/          architecture notes, mode guide, experiment write-up
```

## Documentation index

- [docs/pose-estimation-modes.md](docs/pose-estimation-modes.md) — modes C/D, full keyboard reference, experiment procedure
- [docs/lighting-experiment.md](docs/lighting-experiment.md) — the lighting experiment: method, results, analysis
- [docs/architecture-notes.md](docs/architecture-notes.md) — module map, ownership, the State pattern
