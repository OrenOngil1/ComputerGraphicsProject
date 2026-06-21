# Architecture Notes (AI orientation)

> Dense map of the architecture. Written for a future Claude Code session to
> read first. Verify line numbers against the current code before quoting them —
> structure is stable, exact lines drift.
>
> **Status: all four modes are implemented** — A Navigation, B Picking, C
> Trackers, D Feature Matching — plus a directional lighting model. Modes B and
> D are *manual* (the user supplies the 3D half of each correspondence by
> color-picking the global map); C is automatic (colored fiducials); D's ORB
> only *suggests* the 2D points. For the per-mode walkthrough, controls, and the
> experiments, read **`docs/pose-estimation-modes.md`** (and
> `docs/lighting-experiment.md`); this file is the structural/ownership map.

## Mental model

`main.cpp` is a 3-line entry point; the **composition root is `Application`**
(`src/core/Application.{h,cpp}`). It runs a **unified menu/session loop**: the window
+ shaders are created **once** (a `Window` RAII member + a mesh-less `Renderer`), then
each menu pass swaps the terrain in place via `Renderer::loadTerrain` and runs the
frame loop until the session ends. `Simulation` is a **long-lived `Application` member**,
reset per terrain in `loadTerrain` (so recordings don't bleed across terrains);
callbacks reach it through a `CallbackContext { Simulation*, Renderer* }` (defined in
`input/Callbacks.h`) that is a **local in `run()`** — set on the GLFW user pointer there,
so `Application` holds no pointer into its own members (no self-reference). **Exit:**
Escape sets `sim.returnToMenu` (→ back to menu); `Ctrl+Q` or the OS close button trip
`glfwWindowShouldClose` (→ quit the program). No global state.

## Module map & ownership

- `src/main.cpp` — 3 lines: `Application app; return app.run();`.
- `src/core/Application.{h,cpp}` — the composition root. Three members in **load-bearing
  declaration order**: `Window m_window` (context goes live) → `Simulation m_sim`
  → `Renderer m_renderer` (mesh-less ctor compiles shaders). Ctor registers the 3
  callbacks, seeds the viewports directly (`leftHalf`/`rightHalf`, no user pointer
  needed), and sets persistent GL state. `loadTerrain(path)`: `readTerrain` →
  `m_renderer.loadTerrain` → reposition cameras → reset `pathPoints`/`waypoints` +
  `NavigationState`. `runSession()`: the frame loop
  `tick → clear → renderGlobalView → renderPlayerView → swap/poll`, until
  `glfwWindowShouldClose || returnToMenu`. `run()`: builds the `CallbackContext` **local**,
  points the GLFW user pointer at it, then loops
  `loadTerrain(selectTerrain(...)) → runSession()`, breaking on window-close (quit);
  the menu's "Exit" entry (`exit(0)`) is the other program exit.
- `src/core/Window.{h,cpp}` — RAII platform owner: ctor does glfwInit → 3.3-core
  window → GLAD (throws on failure); dtor does `glfwTerminate`. Declared before
  `Renderer` so the context is live at shader compile and torn down only after
  `~Renderer`.
- `src/core/`
  - `Simulation.h` — `struct Simulation`: `unique_ptr<State> currentState`,
    `terrainSize`, `Mesh mesh`, `View globalView`/`playerView`,
    `vector<glm::vec3> pathPoints`, `vector<Waypoint> waypoints`, and
    `bool returnToMenu` (Escape sets it; the session loop checks it). Out-of-line
    `~Simulation()` (needs complete `State`). The non-owning `CallbackContext { Simulation*,
    Renderer* }` bundle on the window user pointer is defined in `input/Callbacks.h`,
    not here.
  - `Camera.h` — `Camera` (position/target/up + fov/near/far), `Viewport`
    (x/y/w/h), `View {Camera; Viewport;}`, free `leftHalf`/`rightHalf` layout
    helpers, and `Waypoint` (recorded pose: position + target).
  - `Scene.h` — `Vertex {position, color, normal}` (normal added for lighting),
    `Correspondence {worldPos, normalized imagePos + imagePixels() denormalizer}`
    (formerly `PickedPoint`), `Mesh {cols, rows, vertices}` with `center()` /
    `worldPos(id)` helpers (formerly `width`/`height`), plus `Tracker {center,
    radius, color}` (Mode C) and `FramePixels {w, h, rgb}` (a CPU-side viewport
    read-back, top-down RGB) for the vision passes. Plain `struct`s.
  - `Lighting.h` — `DirectionalLight {direction, color, ambient}` and the
    `kLightPresets` table (late-morning / noon / low-warm / overcast) the `L`
    key cycles. The light lives on `Simulation` and survives terrain swaps.
  - `Menu.{h,cpp}` — `selectTerrain(dir)`: console menu over the DEM images in a
    folder; blocks on `std::cin`; trailing "Exit" entry calls `exit(0)`.
  - `Utils.h` — header-only helpers: `randomIndex(n)`, glm↔OpenCV point
    conversions, and `getCameraIntrinsicMatrix(fov,w,h)` — the pinhole **K** for
    PnP, **square pixels (`fx == fy`)**: the aspect is carried by w≠h and the
    principal point, NOT the focal length (a wrong `fx` here once biased every
    pose ~22% on the non-square player viewport).
- `src/render/Renderer.h/.cpp` — **sole owner of GPU resources**: three `Shader`s
  (`m_sceneShader`, `m_pickShader`, `m_pointShader`) + two `GpuMesh` (VA/VB/IB):
  `m_terrain` (swapped per DEM) and `m_sphere` (the shared unit sphere every
  Mode-C tracker draws). Non-copyable. **Mesh-less ctor** compiles the shaders +
  builds the sphere; `loadTerrain` is the sole terrain-upload path.
  - Public: `clear()`, `loadTerrain(Mesh)`, `renderGlobalView`/`renderPlayerView
    (View, Simulation)`; overlay surface `drawPath` / `drawWaypoints` /
    `drawPoints` / `drawTrackers`; PICK support `pickVertex(x,y,View) → id`
    (offscreen id-color pass + `glReadPixels`) and `drawGhost(...)`; and the
    full-frame vision read-backs `captureSceneFrame(View, light)` (lit RGB),
    `captureVertexIdFrame(View)` (per-pixel vertex id), `captureTrackersFrame`
    (flat-color spheres on black) — all offscreen, never swapped.
  - Private `renderScene(Camera, Viewport, DirectionalLight) → mvp`: viewport →
    MVP → draw terrain lit (Lambert + ambient, gated by a `u_Lit` uniform so the
    overlay/capture/pick passes stay unshaded); returns the MVP for the overlay.
- `src/state/` — the State pattern (see below).
- `src/input/`
  - `Callbacks.cpp` — `keyCallback` (Escape → `returnToMenu`; `Ctrl+Q` → quit;
    `L` → cycle the light; else `tryTransition` then `currentState->handleKey`),
    `mouseButtonCallback`, `framebufferSizeCallback`, plus `scrollCallback` /
    `cursorPosCallback` for the **global-map controls** (scroll = zoom,
    middle-drag = pan, right-drag = orbit — these act only over the global view
    and move only the overview camera; the drag state lives on
    `CallbackContext`). The transition machinery (`setState`, `requireWaypoints`,
    `tryTransition`) lives here too.
  - `Movement.*` — `moveCamera(camera, terrainSize, window, dt)`: continuous,
    frame-rate-independent FPS flight. **Polls** held keys (`glfwGetKey`) each
    frame: WASD translate (forward follows pitch), arrows look (pitch clamped),
    `Q`/`E` altitude. Speed scales with `terrainSize` and `dt`.
- `src/vision/` — the OpenCV layer, decoupled from rendering (operates on
  `Correspondence`s and `FramePixels`, never GL):
  - `Pnp.{h,cpp}` — `computeCameraPose(...)` (SQPnP, for the exact correspondences
    of Pick/Trackers) and `computeCameraPoseRansac(..., minInliers)` (for the
    noisier feature matches); both → `optional<Waypoint>`.
  - `TrackerDetection.{h,cpp}` — `findTrackerCentroids(frame, trackers)`: classify
    each pixel against the palette; each color's blob centroid is its 2D point.
  - `FeatureMatching.{h,cpp}` — `detectTopFeatures` (the strongest N ORB keypoints,
    Mode D's suggestions) and `estimatePoseFromFeatures` (run-phase: match the
    live view against the hand-built `FeatureDb` + RANSAC).
- `src/loader/TerrainLoader.*` — `readTerrain(path) → Mesh` from a DEM image.
- `src/engine/` — thin OpenGL 3.3-core wrappers: `Shader`, `VertexArray`,
  `VertexBuffer`, `IndexBuffer`, `VertexBufferLayout`, `Debugger`.

## Control flow

- **Frame loop** (`Application::runSession`): `currentState->tick(sim, window, dt)` (continuous
  movement, dt-scaled), then `renderer.clear()`, then `renderGlobalView` then
  `renderPlayerView`, each: set viewport → build MVP → draw terrain → call the
  current state's `render*Overlay(sim, *this, mvp)`.
- **Key input** (`keyCallback`): ignores releases; **Escape** sets
  `sim.returnToMenu` (ends the session → back to the menu) and **`Ctrl+Q`** quits the
  program (`glfwWindowShouldClose`); then `tryTransition` (global mode hotkeys) runs
  and short-circuits; otherwise `currentState->handleKey`.
- **Mouse input** (`mouseButtonCallback`): middle/right buttons are intercepted
  for the global-map pan/orbit (never reach the mode); left is routed to
  `currentState->handleMouseButton(...)` — `PickState` and the `FeatureMatchState`
  build use it to color-pick a vertex in the global view.
- **Transitions** (`tryTransition` in `Callbacks.cpp`): `R`→Record,
  `Ctrl+R`→Playback, `P`→Pick, `T`→Trackers (prompts for a count), `F`→Feature
  Matching (prompts for a count). Playback/Pick/Feature-Matching are guarded by
  `requireWaypoints`. `L` (cycle light) is handled in `keyCallback` *before*
  `tryTransition`, so it works in any mode. `setState` swaps the pointer then
  calls `onEnter` — the one place onEnter runs.
- **In-mode keys** (per the state headers): Navigation = continuous flight (see
  `Movement.*`); Record = `B` stores a waypoint; Playback = `UP`/`DOWN` step
  waypoints; Pick = click a 2D point then its 3D match, `C` solves PnP; Trackers
  & Feature Matching = `B` capture a timestep, `N`/`M` review (from
  `PoseComparisonState`); Feature Matching also = `G` start the manual database
  build, `X` skip a suggestion during it.
- **Resize** (`framebufferSizeCallback`): recomputes `globalView.viewport =
  leftHalf(w,h)`, `playerView.viewport = rightHalf(w,h)`. Pixels, not screen
  coords (HiDPI). The loop only reads viewports — never recomputes them.

## State pattern (`src/state/`)

- `State.h` — base, all hooks default to no-op so a mode overrides only what it
  uses: `onEnter`, `handleKey`, `tick` (per-frame, dt), `handleMouseButton`
  (receives a `Renderer&` so a mode can trigger a render pass), and
  `renderGlobalOverlay` / `renderPlayerOverlay` (receive a `Renderer&`, **not** a
  `Shader`).
- `States.h/.cpp`:
  - `NavigationState` — `tick` → `moveCamera` (free flight).
  - `RecordState` — `tick` flies + samples path points past a distance threshold;
    `B` stores a waypoint; overlays path+waypoints on the global view.
  - `PlaybackState` — `UP`/`DOWN` step the private `m_index`; snaps the camera to
    the selected waypoint; overlays path+waypoints.
  - `PickState` (Mode 2) — `onEnter` seeds the player camera at a random recorded
    waypoint (the unknown pose to recover). A correspondence is built in two
    clicks: a 2D point in the player view (stored as an aspect-invariant camera
    ray, no vertex pick) then its 3D match color-picked in the global view via
    `renderer.pickVertex`; `C` → `computeCameraPose`. Overlays: 3D points + the
    estimated camera on the map; the 2D observations in the player view, then the
    translucent `drawGhost` once solved.
  - `PoseComparisonState` (`PoseComparisonState.{h,cpp}`) — shared base of the two
    automatic-display modes. Free flight; `B` captures a `(true, computed)` pose
    pair into a `PoseLog`; `N`/`M` review timesteps; the global view draws both
    fly-through paths and the player view the ghost. The one pure virtual is
    `computePose` — the only thing the two subclasses differ on.
  - `TrackersState` (Mode 3, `TrackersState.{h,cpp}`) — `onEnter` scatters N
    colored fiducial spheres (count from a prompt); `computePose` reads back the
    detection frame, `findTrackerCentroids` → 2D points paired with the spheres'
    known 3D centers → PnP. No clicks.
  - `FeatureMatchState` (Mode 4, `FeatureMatchState.{h,cpp}`) — manual. `G` runs an
    interactive build (a `.cpp`-private `BuildScratch` sub-state): step each
    recorded view, ORB suggests the strongest N points one at a time, the user
    color-picks each one's 3D on the map → a hand-anchored `FeatureDb`.
    `computePose` (run-phase `B`) matches the live view's ORB features against
    that DB + RANSAC. `tick`/`handleKey` branch on whether a build is in progress.
- **No `id()`/`Mode` enum** on purpose: prevents reintroducing `switch(id())`.

## Load-bearing invariants (do not break)

1. **`Renderer` owns all GPU resources.** Overlays draw *through* the Renderer;
   the scene shader never leaves its owner.
2. **Teardown order via member order.** In `Application`, `Window` is declared
   **before** `Renderer`, so reverse-order destruction runs `~Renderer` (`glDelete*`)
   before `~Window` (`glfwTerminate()`). `Simulation` owns no GL handles, so its member
   position is safe. Keep it that way (see tripwire below) — and keep `Window`
   declared before `Renderer`.
3. **Transitions live in `Callbacks.cpp`,** not inside states; states never name
   other states.
4. **Camera is read-only render input; Viewport is pure layout.**

## Extension seams

- **Modes 2–4: DONE.** Picking = `PickState` + `Renderer::pickVertex`/`drawGhost`;
  Trackers / Feature-Matching = `PoseComparisonState` subclasses + the `vision/`
  detectors. The color-pick pass still lives **on `Renderer`** (`m_pickShader`);
  if it grows (FBO, depth read-back) the documented move is to extract a dedicated
  `ColorPicker`/`PickPass` rather than widen `Renderer` further.
- **Adding a mode** (the established recipe): a new `State` subclass (or a
  `PoseComparisonState` subclass for the capture/compare display), a transition in
  `tryTransition`, and any CV work behind the `vision/` boundary; reuse `Utils.h`
  intrinsics + `Pnp`.
- **Terrain swap:** the session loop is **unified** — `Application::loadTerrain`
  calls `Renderer::loadTerrain(newMesh)` to swap the mesh in place; the window and
  compiled shaders persist across menu picks.

## Gotchas / tripwires

- **GL-handle-outside-Renderer tripwire:** the moment a `State` or `Simulation`
  member owns a raw GL handle, invariant #2's safety for `Simulation` breaks (its
  destruction relative to `~Window`/`glfwTerminate()` is governed by member position).
  Route GL handles through the `Renderer` facade, or give the owner a position after
  `Window` in `Application`. The `Window` RAII guard that makes this tractable now
  exists (`src/core/Window.{h,cpp}`).


## Refactor direction (current)

- **Shipped:** the unified session loop + `Application` class + mesh-less `Renderer`
  (the `application-class-refactor.md` design, with its `optional`-removing
  refinement). `main-refactor.md`'s fork resolved to "unified ⇒ class"; the free
  functions it proposed became `Application`'s ctor body / private methods.
- **Deferred — `Simulation` split (Axis B):** at this size every `State` legitimately
  uses cross-cutting slices (camera + terrainSize + recording at once), so extracting
  a `Recording` struct doesn't narrow signatures. Revisit when Mode 3 adds
  correspondence data with real consumers.
- **Deferred — `Renderer` pick-pass / `ColorPicker` extraction (Axis C):** single
  dispatch is correct as-is; defer until Modes 3/4 press on it.

## Cross-references

- **`docs/pose-estimation-modes.md`** — the four modes, full controls, and the
  experiments (read for *behavior*; this file is *structure*).
- **`docs/lighting-experiment.md`** — the lighting / feature-stability experiment.
- `docs/legacy-import.md` — human-facing end-of-branch snapshot ("where the project is now").
- `docs/refactor-summary.md` — the earlier `oop`-refactor overview.
- `docs/main-refactor.md` — the main.cpp decomposition fork (resolved → unified).
- `docs/renderer-class-refactor.md`, `state-pattern-refactor.md` — per-refactor
  decision notes. `application-class-refactor.md` — **implemented** (see above).
- `docs/picking-click-drift.md`, `wslg-ghost-window.md` — known issues / env notes.
