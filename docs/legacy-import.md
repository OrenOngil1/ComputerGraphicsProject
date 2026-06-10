# Where the project is now

A human-readable snapshot of the drone-sim / PnP project as it stands on branch
`legacy-import`. For the dense module map see `architecture-notes.md`; for the
original refactor story see `refactor-summary.md`.

## What it does today

Run `./bin/drone_sim` and you get a console menu of terrains (DEM images). Pick
one and a split-screen window opens: **global view** (left) watching the terrain
from above, **player view** (right) from a camera you fly. Escape returns to the
menu (picking another terrain swaps it in place — the window and shaders persist);
`Ctrl+Q`, the menu's "Exit", or the window's OS close button quit the program.

Four things work end to end:

- **Navigation** — free FPS flight of the player camera: WASD to move (forward
  follows where you look), arrow keys to look, Shift + `.` / `,` for altitude.
  Movement is smooth and frame-rate-independent.
- **Record** (`R`) — fly and the path is traced on the global view; `B` drops a
  waypoint (a saved camera pose).
- **Playback** (`Ctrl+R`) — step through the recorded waypoints with `UP` / `DOWN`;
  the camera snaps to each, highlighted on the global map.
- **Pick / Mode 2** (`P`) — the camera is dropped at a random recorded waypoint
  (the "unknown" pose). Left-click terrain points to build 2D-3D correspondences,
  press `C` to solve **PnP** (OpenCV). The estimate shows as a marker on the global
  view and a translucent "ghost" terrain over the true player view — the closer
  the two line up, the better the pose estimate.

## What's next

- **Mode 3 (Trackers)** — automated pose from uniquely-colored 3D spheres.
- **Mode 4 (2D Feature Matching)** — ORB/SIFT + matcher correspondences under
  varying lighting. Both are new `State` subclasses reusing the `vision/` (PnP)
  layer.

## Known issues / notes (recorded, not blocking)

- `picking-click-drift.md` — a fast click-then-move can record the moved cursor
  position; documented fix (click-vs-drag thresholding) deferred.
- `wslg-ghost-window.md` — a stuck run can leave a ghost window under WSLg; fix is
  `wsl --shutdown` (no reboot needed).

## Architecture refactor (shipped)

- The bloated `main.cpp` was resolved by moving to a **unified session loop** behind
  an **`Application` class** (`src/core/Application.{h,cpp}`) + a `Window` RAII wrapper
  (`src/core/Window.{h,cpp}`). Window + shaders are created once; terrain swaps in
  place via `Renderer::loadTerrain` (now mesh-less ctor). Member declaration order
  encodes GL teardown ordering, so the old load-bearing `{ }` scope and the proposed
  `optional<Renderer>` are both gone. Two-signal exit: Escape (`returnToMenu`) → menu,
  OS close → quit. See `application-class-refactor.md` (implemented) and
  `main-refactor.md` (the fork that resolved to "unified ⇒ class").

## Build

```
make            # → bin/drone_sim
make clean
./bin/drone_sim
```
