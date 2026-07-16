# Architecture Notes

A structural map of the codebase: what owns what, how a frame flows, and the
invariants the design leans on. For *behavior* - the four modes, controls, and
the experiments, read [pose-estimation-modes.md](pose-estimation-modes.md) and
[lighting-experiment.md](lighting-experiment.md); this file is about *structure*.

## Mental model

`main.cpp` is a 3-line entry point; the **composition root is `Application`**
(`src/core/Application.{h,cpp}`). It runs a **unified menu/session loop**: the
window and shaders are created **once** (a `Window` RAII member + a mesh-less
`Renderer`), then each menu pass swaps the terrain in place via
`Renderer::loadTerrain` and runs the frame loop until the session ends.

`Simulation` is a long-lived `Application` member, reset per terrain in
`loadTerrain` so recordings don't bleed across terrains. Callbacks reach it
through a `CallbackContext { Simulation*, Renderer*, OrbitController }`
(defined in `input/Callbacks.h`) that lives on `run()`'s stack and is set on
the GLFW window user pointer.

**Exit paths:** Escape sets `sim.returnToMenu` (back to the terrain menu);
`Ctrl+Q` or the OS close button trip `glfwWindowShouldClose` (quit); the menu's
"Exit" entry makes `selectTerrain` return `nullopt`, which `run()` treats as
quit. All three unwind normally, so destructors run. No global state.

## Module map & ownership

- `src/main.cpp` — `Application app; return app.run();` inside the one
  try/catch error boundary.
- `src/core/Application.{h,cpp}` — the composition root. Three members in
  **load-bearing declaration order**: `Window m_window` (context goes live) →
  `Simulation m_sim` → `Renderer m_renderer` (ctor compiles shaders). The ctor
  registers callbacks, seeds the split-screen viewports, and sets persistent GL
  state. `loadTerrain(path)`: read mesh → upload to renderer → reposition
  cameras → reset per-terrain state. `runSession()`: the frame loop.
- `src/core/Window.{h,cpp}` — RAII platform owner: ctor does glfwInit →
  3.3-core window → GLAD (throws on failure); dtor does `glfwTerminate`.
- `src/core/` data types:
  - `Simulation.h` — the per-session shared state: the active `State`, the
    mesh, the two `View`s, the recording (`pathPoints` + `waypoints`), the
    light preset, and the `returnToMenu` flag.
  - `Camera.h` — `Camera` (eye pose + lens), `View` (camera + viewport pair),
    `Waypoint` (recorded pose), and `viewProjection` — the one definition of
    the projection the renderer draws with, which `Pnp.cpp`'s intrinsics must
    mirror (a headless check holds the two together).
  - `Viewport.h` — `Viewport` (screen rectangle, pure layout) and the
    `leftHalf` / `rightHalf` split-screen helpers.
  - `Scene.h` — `Vertex`, `Correspondence` (3D point + normalized 2D
    observation), `Mesh` (the height grid, with the `center()` /
    `worldPos(id)` centering authority), `Tracker`, and `FramePixels` (a
    CPU-side viewport read-back, top-down RGB — the renderer→vision hand-off).
  - `Lighting.h` — `DirectionalLight` + the `kLightPresets` table the `L` key
    cycles. Lives on `Simulation` and survives terrain swaps (the Mode D
    experiment depends on that).
  - `Menu.{h,cpp}` — `selectTerrain(dir)`: console menu over the DEM images in
    a folder; `nullopt` = the user chose Exit.
  - `Random.h` — `randomIndex(n)`.
- `src/render/Renderer.{h,cpp}` — **sole owner of GPU resources**: three
  `Shader`s (scene, pick, point), two `GpuMesh`es (`m_terrain`, swapped per
  DEM; `m_sphere`, the unit sphere every tracker draw reuses), and the
  `SkyPass`. The sky draws after the terrain but before the overlays, and only
  in the visible views — the vision captures and the pick pass never see it.
  Public surface:
  the two per-view draws, the overlay primitives (`drawPath`, `drawWaypoints`,
  `drawPoints`, `drawTrackers`, `drawGhost`), the color-pick pass
  (`pickVertex`), and the vision read-backs (`captureSceneFrame`,
  `captureTrackersFrame`) — all capture passes render to the back buffer and
  never swap, so they are invisible.
- `src/render/GpuMesh.{h,cpp}` — the GPU-resident mesh bundle (VAO + VBO +
  IBO) and its builders (`uploadTerrain`, `buildSphereMesh`,
  `buildSkyboxCube`). Construction only; drawing stays in `Renderer`.
- `src/render/PickEncoding.h` — both directions of the pick pass's id↔color
  packing, side by side in one header; the pick shader just passes the baked
  per-vertex attribute through, so the packing rule has a single home.
- `src/render/SkyPass.{h,cpp}` — the per-preset skybox: its own shader, the
  unit cube, and a lazy cache of GL cubemap textures (a preset whose skybox
  fails to load warns once and keeps the clear-color sky). The one home of
  manually managed GL texture lifetime; the asset root is injected at
  construction, so no draw code composes paths.
- `src/state/` — one `State` subclass per mode (see below).
- `src/input/`
  - `Callbacks.{h,cpp}` — GLFW glue: the callbacks, the transition machinery
    (`setState`, `requireWaypoints`, `tryTransition`), and
    `pollMovementIntent`. The global-map controls (scroll = zoom, middle-drag =
    pan, right-drag = orbit) are intercepted here and never reach the modes.
  - `CameraControls.{h,cpp}` — the **pure, GLFW-free camera verbs**
    (`zoom`/`pan`/`orbit`/`fly`), the `MovementIntent` struct, and the
    `OrbitController` drag lifecycle. Window-free by design, so all of it is
    unit-tested headlessly.
- `src/vision/` — the OpenCV layer, decoupled from rendering: it operates on
  `Correspondence`s and `FramePixels`, never GL.
  - `Pnp.{h,cpp}` — `computeCameraPose` (SQPnP, exact correspondences) and
    `computeCameraPoseRansac(..., minInliers)` (noisy feature matches). The
    pinhole intrinsics **K** is a file-local detail here — square pixels
    (`fx == fy`), aspect carried by width/height, matching what
    `glm::perspective` renders.
  - `TrackerDetection.{h,cpp}` — `findTrackerCentroids`: classify each
    read-back pixel against the tracker palette; each color's blob centroid is
    its 2D point.
  - `FeatureMatching.{h,cpp}` — `detectTopFeatures` (Mode D's suggestions) and
    `estimatePoseFromFeatures` (match the live view against the hand-built
    `FeatureDb`, then RANSAC PnP).
- `src/loader/TerrainLoader.{h,cpp}` — DEM image → `Mesh` (heights, elevation
  ramp colors, central-difference normals).
- `src/loader/SkyboxLoader.{h,cpp}` — skybox folder → `CubemapFaces` (six
  validated square BGR faces, named by GL face order).
- `external/` — vendored third-party code built from source, each component its
  own library target; not modified by this project.
  - `external/engine/` — the BasicOpenGL course toolkit: thin OpenGL wrappers
    (`Shader`, `VertexArray`, ...).
  - `external/glad/` — the glad OpenGL loader, in its upstream layout.
- `include/` — vendored header-only libraries (glm); consumed via the include
  path alone, nothing to build.

## Control flow

- **Frame loop** (`Application::runSession`): `currentState->tick(sim, window,
  dt)` (continuous movement, dt-scaled), then clear, then each view: set
  viewport → build MVP → draw terrain lit → call the state's
  `render*Overlay(sim, renderer, mvp)`.
- **Key input** (`keyCallback`): ignores releases; Escape / `Ctrl+Q` / `L`
  are handled globally, then `tryTransition` (mode hotkeys) short-circuits,
  otherwise the event routes to `currentState->handleKey`.
- **Mouse input** (`mouseButtonCallback`): middle/right are intercepted for the
  global-map controls; left routes to `currentState->handleMouseButton` —
  `PickState` and the `FeatureMatchState` build use it to color-pick a vertex.
- **Transitions** (`tryTransition`): `R` → Record, `Ctrl+R` → Playback, `P` →
  Pick, `T` → Trackers (count prompt), `F` → Feature Matching (count prompt).
  Playback/Pick/Feature-Matching are guarded by `requireWaypoints`. `setState`
  swaps the pointer then calls `onEnter` — the one place `onEnter` runs.
- **Resize** (`framebufferSizeCallback`): recomputes the two viewports from the
  new framebuffer size (pixels, not screen coords — HiDPI). The render loop
  only reads viewports, never recomputes them.

## The State pattern (`src/state/`)

`State.h` is the base: every hook defaults to a no-op, so a mode overrides only
what it uses (`onEnter`, `handleKey`, `tick`, `handleMouseButton`, the two
overlays). There is deliberately **no `id()`/mode enum** — that would invite a
`switch(state->id())`, which is exactly what the pattern exists to eliminate. Transitions are
an app-level concern in `Callbacks.cpp`; states never name other states.

- `NavigationState` — free flight.
- `RecordState` — flight + path sampling; `B` stores a waypoint.
- `PlaybackState` — `UP`/`DOWN` step the recorded waypoints.
- `PickState` (Mode B) — seeds the camera at a random waypoint (the unknown
  pose); two clicks per correspondence (2D in the player view — stored as an
  aspect-invariant camera ray so picks survive a resize — then 3D color-picked
  on the map); `C` solves PnP; overlays markers, the estimate, and the ghost.
- `PoseComparisonState` — shared base of the two automatic-display modes:
  flight, `B` captures a `(true, computed)` pose pair into a `PoseLog`,
  `N`/`M` review, dual-path + ghost display. The one pure virtual is
  `computePose`.
- `TrackersState` (Mode C) — scatters colored fiducial spheres; `computePose` =
  detection-frame read-back → blob centroids → PnP. No clicks.
- `FeatureMatchState` (Mode D) — `G` runs the interactive database build (a
  `.cpp`-private `BuildScratch` sub-state): per recorded view, ORB suggests
  points one at a time and the user anchors each on the map. `computePose`
  matches the live view against that database + RANSAC.

## Load-bearing invariants

1. **`Renderer` owns all GPU resources.** Overlays draw *through* the Renderer;
   the shaders never leave their owner.
2. **Teardown order via member order.** In `Application`, `Window` is declared
   before `Renderer`, so reverse-order destruction runs `~Renderer`
   (`glDelete*`) before `~Window` (`glfwTerminate`). Corollary: the moment any
   `State` or `Simulation` member owns a raw GL handle, this breaks —
   route GL work through the `Renderer` facade instead.
3. **Transitions live in `Callbacks.cpp`,** not inside states.
4. **`Camera` is read-only render input; `Viewport` is pure layout.**
5. **One centering authority.** The renderer bakes `-Mesh::center()` into the
   uploaded vertices; every consumer mapping a stored vertex into the rendered
   world goes through `Mesh::worldPos`. The render and vision sides cannot
   drift apart.

## Adding a mode (the established recipe)

A new `State` subclass (or a `PoseComparisonState` subclass if it captures and
compares poses), a transition case in `tryTransition`, and any CV work behind
the `vision/` boundary. If the color-pick pass ever grows (FBO, depth
read-back), extract a dedicated `PickPass` rather than widening `Renderer`.

## Testing

`tests/` builds one headless binary (no window, no GL) from per-topic files:
terrain normals, tracker centroids, both PnP solvers, the camera verbs, the
pick-id encoding round trip, the split-screen viewport layout, the pose-review
log cursor, the ORB suggestion step (`detectTopFeatures`, including its
keypoint↔descriptor row alignment), and the render↔vision camera-model contract
(`viewProjection` and the viewing-ray mapping vs. an independent pinhole) —
each against synthetic inputs with known answers. The PnP round-trip projects
through an independently-derived square-pixel pinhole on a non-square viewport,
so intrinsics mistakes (an aspect factor folded into the focal length) fail the
suite.
