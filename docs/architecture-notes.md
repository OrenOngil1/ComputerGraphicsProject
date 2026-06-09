# Architecture Notes (AI orientation)

> Dense map of the current architecture on branch `legacy-import` (legacy
> features — movement, picking, menu — ported onto the post-`oop` refactor).
> Written for a future Claude Code session to read first. Verify line numbers
> against the current code before quoting them — structure is stable, exact
> lines drift.

## Mental model

`main.cpp` is the **composition root**, wrapped in an **outer menu/session loop**:
each pass picks a terrain (console menu), brings up GL, runs the frame loop until
the window closes (Escape or OS close), tears GL down, and re-enters the menu.
`AppState` is a **fresh local per terrain** (so recordings don't bleed across
terrains); callbacks reach it through an `AppContext { AppState*, Renderer* }`
stored on the GLFW user pointer. There is **no global state** and (deliberately)
**no `Application` class** — see Deferred below.

## Module map & ownership

- `src/main.cpp`
  - `initGL(AppContext&)` → `GLFWwindow*`: GLFW init → 3.3 core window → GLAD →
    persistent GL state. Wires the GLFW **user pointer** to the `AppContext` and
    sets `keyCallback` / `mouseButtonCallback` / `framebufferSizeCallback`.
  - `main()`: an **outer `while(true)`** session loop — `selectTerrain` (menu) →
    `readTerrain` → build `AppState` (two `View`s, initial `NavigationState`) →
    `initGL` → an **inner `{ }` scope** constructs `Renderer` and loops
    `tick → clear → renderGlobalView → renderPlayerView → swap/poll` → `glfwTerminate`.
    The menu's "Exit" entry (`exit(0)`) is the only program exit.
- `src/core/`
  - `AppState.h` — `struct AppState`: `unique_ptr<State> currentState`,
    `terrainSize`, `Mesh mesh`, `View globalView`/`playerView`,
    `vector<glm::vec3> pathPoints`, `vector<Waypoint> waypoints`. Out-of-line
    `~AppState()` (needs complete `State`). `AppContext { AppState*, Renderer* }`
    (the non-owning bundle on the window user pointer) lives here too.
  - `Camera.h` — `Camera` (position/target/up + fov/near/far), `Viewport`
    (x/y/w/h), `View {Camera; Viewport;}`, free `leftHalf`/`rightHalf` layout
    helpers, and `Waypoint` (recorded pose: position + target).
  - `Scene.h` — `Vertex {position,color}`, `PickedPoint {worldPos,imagePos}`,
    `Mesh {width,height,vertices}` (plain `struct`s).
  - `Menu.{h,cpp}` — `selectTerrain(dir)`: console menu over the DEM images in a
    folder; blocks on `std::cin`; trailing "Exit" entry calls `exit(0)`.
  - `Utils.h` — header-only helpers: `randomIndex(n)`, glm↔OpenCV point
    conversions, and `getCameraIntrinsicMatrix(fov,w,h)` (the pinhole **K** for PnP).
- `src/render/Renderer.h/.cpp` — **sole owner of GPU resources**: three `Shader`s
  (`m_sceneShader`, `m_pickShader`, `m_pointShader`) + `TerrainGpu m_terrain`
  (VA/VB/IB). Non-copyable.
  - Public: `clear()`, `loadTerrain(Mesh)` (swap buffers, reuse shader),
    `renderGlobalView(View, AppState)`, `renderPlayerView(View, AppState)`;
    overlay surface `drawPath` / `drawWaypoints` / `drawPoints`; PICK support
    `pickVertex(x,y,View) → id` (offscreen id-color pass + `glReadPixels`) and
    `drawGhost(...)` (terrain re-drawn translucent from an estimated pose).
  - Private `renderScene(Camera, Viewport) → mvp`: viewport → MVP → draw
    terrain; returns the MVP for the active mode's overlay.
- `src/state/` — the State pattern (see below).
- `src/input/`
  - `Callbacks.cpp` — `keyCallback` (Escape → close window; else `tryTransition`
    then `currentState->handleKey`), `mouseButtonCallback` (→ `currentState`),
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

- **Frame loop** (`main`): `currentState->tick(appState, window, dt)` (continuous
  movement, dt-scaled), then `renderer.clear()`, then `renderGlobalView` then
  `renderPlayerView`, each: set viewport → build MVP → draw terrain → call the
  current state's `render*Overlay(appState, *this, mvp)`.
- **Key input** (`keyCallback`): ignores releases; **Escape** closes the window
  (ends the loop → back to the menu); `tryTransition` (global mode hotkeys) runs
  next and short-circuits; otherwise `currentState->handleKey`.
- **Mouse input** (`mouseButtonCallback`): routed unconditionally to
  `currentState->handleMouseButton(appState, *renderer, window, button, action)`
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
    at a random recorded waypoint (the unknown pose to recover); left-click →
    `renderer.pickVertex` builds a 2D-3D `PickedPoint`; `C` → `computeCameraPose`
    (PnP). Global overlay draws picked points + estimated camera; player overlay
    draws the translucent `drawGhost` from the estimated pose vs. the true view.
- **No `id()`/`Mode` enum** on purpose: prevents reintroducing `switch(id())`.

## Load-bearing invariants (do not break)

1. **`Renderer` owns all GPU resources.** Overlays draw *through* the Renderer;
   the scene shader never leaves its owner.
2. **Teardown order.** `Renderer` lives in an inner scope in `main()` so
   `~Renderer` (calls `glDelete*`) runs **before** `glfwTerminate()`. `AppState`
   is destroyed *after* `glfwTerminate()` — safe only because it owns no GL
   handles. Keep it that way (see tripwire below).
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
- **Terrain swap:** the menu currently recreates the window/`Renderer` per terrain
  (the "primitive" loop). `Renderer::loadTerrain(newMesh)` exists for the
  alternative in-place swap, if the session loop is later unified.

## Gotchas / tripwires

- **GL-handle-outside-Renderer tripwire:** the moment a `State` or `AppState`
  member owns a raw GL handle, invariant #2 breaks (it'd be destroyed after
  `glfwTerminate()`). Then either route it through the `Renderer` facade, or
  introduce a minimal RAII window/context guard *then* (constructed before
  `Renderer`). This is exactly why the `GLContext`/`Application` classes are
  deferred — see `docs/application-class-refactor.md` (deferred).


## Refactor direction (current)

- **The active proposal is `docs/main-refactor.md`** — extract a few free
  functions out of the now-bloated `main.cpp` (createWindow / configureGLState /
  attachContext / configureViews / runFrameLoop). Small, no new class.
- **`docs/application-class-refactor.md` is deprecated/deferred — do NOT pitch it.**
  An `Application` (and `GLContext` window guard) would be over-engineering at this
  scale; `Renderer` remains the sole GPU owner so no lifetime bomb forces it.
- Also deferred: regrouping scene/recording fields off `AppState`.

## Cross-references

- `docs/legacy-import.md` — human-facing end-of-branch snapshot ("where the project is now").
- `docs/refactor-summary.md` — the earlier `oop`-refactor overview.
- `docs/main-refactor.md` — **active** main.cpp decomposition proposal.
- `docs/renderer-class-refactor.md`, `state-pattern-refactor.md` — per-refactor
  decision notes. `application-class-refactor.md` — deferred (see above).
- `docs/picking-click-drift.md`, `wslg-ghost-window.md` — known issues / env notes.
