# Architecture Notes (AI orientation)

> Dense map of the current architecture on branch `legacy-import` (legacy
> features — movement, picking, menu — ported onto the post-`oop` refactor).
> Written for a future Claude Code session to read first. Verify line numbers
> against the current code before quoting them — structure is stable, exact
> lines drift.

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
  - `Scene.h` — `Vertex {position,color}`, `Correspondence {worldPos, normalized
    imagePos + imagePixels() denormalizer}` (formerly `PickedPoint`),
    `Mesh {width,height,vertices}` (plain `struct`s).
  - `Menu.{h,cpp}` — `selectTerrain(dir)`: console menu over the DEM images in a
    folder; blocks on `std::cin`; trailing "Exit" entry calls `exit(0)`.
  - `Utils.h` — header-only helpers: `randomIndex(n)`, glm↔OpenCV point
    conversions, and `getCameraIntrinsicMatrix(fov,w,h)` (the pinhole **K** for PnP).
- `src/render/Renderer.h/.cpp` — **sole owner of GPU resources**: three `Shader`s
  (`m_sceneShader`, `m_pickShader`, `m_pointShader`) + `TerrainGpu m_terrain`
  (VA/VB/IB). Non-copyable. **Mesh-less ctor** compiles the shaders only;
  `loadTerrain` is the sole terrain-upload path.
  - Public: `clear()`, `loadTerrain(Mesh)` (swap buffers, reuse shader),
    `renderGlobalView(View, Simulation)`, `renderPlayerView(View, Simulation)`;
    overlay surface `drawPath` / `drawWaypoints` / `drawPoints`; PICK support
    `pickVertex(x,y,View) → id` (offscreen id-color pass + `glReadPixels`) and
    `drawGhost(...)` (terrain re-drawn translucent from an estimated pose).
  - Private `renderScene(Camera, Viewport) → mvp`: viewport → MVP → draw
    terrain; returns the MVP for the active mode's overlay.
- `src/state/` — the State pattern (see below).
- `src/input/`
  - `Callbacks.cpp` — `keyCallback` (Escape → `returnToMenu`; `Ctrl+Q` → quit; else
    `tryTransition` then `currentState->handleKey`), `mouseButtonCallback` (→ `currentState`),
    `framebufferSizeCallback`, and the transition machinery (`setState`,
    `requireWaypoints`, `tryTransition`).
  - `Movement.*` — `moveCamera(camera, terrainSize, window, dt)`: continuous,
    frame-rate-independent FPS flight. **Polls** held keys (`glfwGetKey`) each
    frame: WASD translate (forward follows pitch), arrows look (pitch clamped),
    Shift+`.`/`,` altitude. Speed scales with `terrainSize` and `dt`.
- `src/vision/Pnp.{h,cpp}` — `computeCameraPose(pickedPoints, fov, w, h) →
  optional<Waypoint>`: solves Perspective-n-Point (OpenCV) from ≥4 2D-3D
  correspondences; the CV layer, decoupled from rendering.
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
- **Mouse input** (`mouseButtonCallback`): routed unconditionally to
  `currentState->handleMouseButton(sim, *renderer, window, button, action)`
  — only `PickState` reacts (left-click → pick a vertex).
- **Transitions** (`tryTransition` in `Callbacks.cpp`): `R`→Record,
  `Ctrl+R`→Playback, `P`→Pick. Playback/Pick are guarded by `requireWaypoints`.
  `setState` swaps the pointer then calls `onEnter` — the one place onEnter runs.
- **In-mode keys** (per `States.h`): Navigation = continuous flight (see
  `Movement.*`); Record = `B` stores a waypoint; Playback = `UP`/`DOWN` step
  waypoints; Pick = left-click adds a correspondence, `C` solves PnP.
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
  - `PickState` (**fully implemented**, Mode 2) — `onEnter` seeds the player camera
    at a random recorded waypoint (the unknown pose to recover). A correspondence is
    built in two clicks: a 2D point in the player view (stored as a normalized
    `imagePos`, no vertex pick) then its 3D match color-picked in the global view (the
    "map") via `renderer.pickVertex`; `C` → `computeCameraPose` (PnP). Global overlay
    draws the 3D points on the map + estimated camera; player overlay draws the 2D
    observations where the user clicked, then the translucent `drawGhost` from the
    estimated pose vs. the true view once solved.
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

## Extension seams (Modes 3–4)

- **Mode 2 (Picking): DONE.** Built as `PickState` + `Renderer::pickVertex` /
  `drawGhost` / `drawPoints` + `vision/Pnp`. The pick pass currently lives **on
  `Renderer`** (`m_pickShader`); if it grows (FBO, depth read-back) the documented
  move is still to extract a dedicated `ColorPicker`/`PickPass` rather than widen
  `Renderer` further.
- **Modes 3/4 (Trackers / 2D feature matching): NEXT.** New `State` subclasses;
  reuse `vision/Pnp` + `Utils.h` intrinsics; keep OpenCV/CV pipeline code
  decoupled from rendering (the `vision/` boundary).
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

- `docs/legacy-import.md` — human-facing end-of-branch snapshot ("where the project is now").
- `docs/refactor-summary.md` — the earlier `oop`-refactor overview.
- `docs/main-refactor.md` — the main.cpp decomposition fork (resolved → unified).
- `docs/renderer-class-refactor.md`, `state-pattern-refactor.md` — per-refactor
  decision notes. `application-class-refactor.md` — **implemented** (see above).
- `docs/picking-click-drift.md`, `wslg-ghost-window.md` — known issues / env notes.
