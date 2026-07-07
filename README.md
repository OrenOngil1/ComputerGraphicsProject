# 2D–3D Pose Estimation on DEM Terrain

[![CI](https://github.com/OrenOngil1/ComputerGraphicsProject/actions/workflows/ci.yml/badge.svg)](https://github.com/OrenOngil1/ComputerGraphicsProject/actions/workflows/ci.yml)

An interactive 3D graphics application in **C++ / OpenGL / OpenCV** that
simulates and solves the **Perspective-n-Point (PnP)** problem: recovering a
camera's 3D pose (position + orientation) from a 2D image. The world is a 3D
terrain built from a **Digital Elevation Map**: a greyscale image whose pixel
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
*producing* those correspondences, from fully manual to fully automatic.

## The four modes

| Mode | Key | How correspondences are made | Demonstrates |
| ---- | --- | ---------------------------- | ------------ |
| **A - Navigation** | - | none (fly, record path `R`, play back `Ctrl+R`) | Dual-view free-flight camera + path recording |
| **B - Picking** | `P` | *manual* - click a 2D point in the camera view, then its 3D match on the map | Human-in-the-loop PnP (the baseline) |
| **C - Trackers** | `T` | *fiducial* - uniquely coloured spheres with known 3D centres; find each colour's blob, its centroid is the 2D point | Automatic correspondence with trivial data association |
| **D - Feature Matching** | `F` | *ORB-assisted manual* - ORB suggests salient points one at a time; the user hand-places each on the map to build the database; the run-phase then matches against it | The markerless case, anchored by hand |

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

### From a terminal

> **Run from the repository root** - the app loads `assets/` relative to the
> working directory.

Generator + toolchain choices live in `CMakePresets.json` (presets `linux` and
`windows`), so configuring is a single command:

```bash
# Linux / macOS
cmake --preset linux
cmake --build --preset linux
ctest --preset linux              # headless math checks
./build/linux/bin/drone_sim
```

```powershell
# Windows — from a VS 2022 developer shell
# ("Developer PowerShell for VS 2022" in the Start menu)
cmake --preset windows
cmake --build --preset windows
ctest --preset windows            # headless math checks
./build/windows/bin/drone_sim.exe
```

Each preset owns its own tree under `build/` (`build/linux/`, `build/windows/`),
so builds for different platforms coexist in one checkout — handy when the same
working copy is built both natively on Windows and from WSL.

> The `windows` preset reads the vcpkg toolchain from `$env{VCPKG_ROOT}`. A VS
> Developer environment provides that variable automatically; outside one, set
> it to your vcpkg checkout (e.g. `C:\vcpkg`).

### From VS Code (CMake Tools)

Install the **CMake Tools** extension, pick the **windows** (or **linux**)
configure preset from the status bar, then use its **Configure / Build / Debug /
Run Tests** actions, no terminal needed. `.vscode/` is pre-wired: CMake Tools
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

`ctest --preset linux` / `ctest --preset windows` (or CMake Tools' **Run
Tests**) builds and runs the checks in `tests/` (no window, no GPU), one file
per topic: the geometry math (terrain normals, camera controls, viewport
layout), the vision steps (tracker blob centroids, ORB feature suggestion, both
PnP solvers including RANSAC outlier rejection), and the cross-layer contracts
(color-pick id round trip, the render↔vision camera model), each on synthetic
inputs with known answers. The camera-model checks project through an
independent square-pixel pinhole on a non-square viewport, so focal-length /
aspect mistakes in the intrinsics fail the suite.

## Repository layout

```
src/core/      composition root, scene state, camera, lighting
src/state/     one State per mode (Navigation, Pick, Trackers, FeatureMatch)
src/render/    Renderer; all GPU work and read-back captures
src/vision/    OpenCV: PnP solvers, blob detection, ORB feature matching
src/loader/    DEM image -> terrain mesh + normals
external/      vendored code built from source: BasicOpenGL toolkit, glad (do not modify)
include/       vendored header-only libraries: glm (do not modify)
assets/        shaders, terrain DEMs, skyboxes
tests/         headless math checks (ctest)
scripts/       per-OS dependency setup (setup.sh, setup.ps1)
docs/          architecture notes, mode guide, experiment write-up
```

## Documentation index

- [docs/pose-estimation-modes.md](docs/pose-estimation-modes.md) - modes C/D, full keyboard reference, experiment procedure
- [docs/lighting-experiment.md](docs/lighting-experiment.md) - the lighting experiment: method, results, analysis
- [docs/architecture-notes.md](docs/architecture-notes.md) - module map, ownership, the State pattern

## License

This project is released under the **MIT License** - see [LICENSE](LICENSE).

It bundles and depends on third-party components (GLM, glad, GLFW, OpenCV, and a
course-issued OpenGL toolkit), each under its own license; see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
