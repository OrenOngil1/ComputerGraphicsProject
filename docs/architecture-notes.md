# Architecture Notes (AI orientation)

> Dense map of the post-refactor architecture on branch `oop`, written for a
> future Claude Code session to read first. Verify line numbers against the
> current code before quoting them — structure is stable, exact lines drift.

## Mental model

`main.cpp` is the **composition root**. It owns a single local `AppState`, sets
up GL, and runs the frame loop. Everything else is small, single-responsibility
types reached from there. There is **no global state** and (deliberately) **no
`Application` class** — see Deferred below.

## Module map & ownership

- `src/main.cpp`
  - `initGL(AppState&)` → `GLFWwindow*`: GLFW init → 3.3 core window → GLAD →
    persistent GL state. Wires the GLFW **user pointer** to `appState` and sets
    `keyCallback` / `framebufferSizeCallback`.
  - `main()`: builds `AppState` (terrain via `readTerrain`, two `View`s, initial
    `NavigationState`), then in an **inner `{ }` scope** constructs `Renderer`
    and loops `clear → renderGlobalView → renderPlayerView → swap/poll`.
- `src/core/`
  - `AppState.h` — `struct AppState`: `unique_ptr<State> currentState`,
    `terrainSize`, `Mesh mesh`, `View globalView`/`playerView`,
    `vector<glm::vec3> pathPoints`, `vector<Waypoint> waypoints`. Out-of-line
    `~AppState()` (needs complete `State`).
  - `Camera.h` — `Camera` (position/target/up + fov/near/far), `Viewport`
    (x/y/w/h), `View {Camera; Viewport;}`, free `leftHalf`/`rightHalf` layout
    helpers, and `Waypoint` (recorded pose: position + target).
  - `Scene.h` — `Vertex {position,color}`, `Mesh {width,height,vertices}`.
- `src/render/Renderer.h/.cpp` — **sole owner of GPU resources**: `Shader
  m_sceneShader` + `TerrainGpu m_terrain` (VA/VB/IB). Non-copyable.
  - Public: `clear()`, `loadTerrain(Mesh)` (swap buffers, reuse shader),
    `renderGlobalView(View, AppState)`, `renderPlayerView(View, AppState)`,
    and overlay surface `drawPath(...)` / `drawWaypoints(...)`.
  - Private `renderScene(Camera, Viewport) → mvp`: viewport → MVP → draw
    terrain; returns the MVP for the active mode's overlay.
- `src/state/` — the State pattern (see below).
- `src/input/Callbacks.cpp` — `keyCallback`, `framebufferSizeCallback`, and the
  transition machinery (`setState`, `requireWaypoints`, `tryTransition`).
  `Movement.*` — camera movement helpers.
- `src/loader/TerrainLoader.*` — `readTerrain(path) → Mesh` from a DEM image.
- `src/engine/` — thin OpenGL 3.3-core wrappers: `Shader`, `VertexArray`,
  `VertexBuffer`, `IndexBuffer`, `VertexBufferLayout`, `Debugger`.

## Control flow

- **Frame loop** (`main`): `renderer.clear()`, then `renderGlobalView` then
  `renderPlayerView`, each: set viewport → build MVP → draw terrain → call the
  current state's `render*Overlay(appState, *this, mvp)`.
- **Input** (`keyCallback`): ignores releases; `tryTransition` (global mode
  hotkeys) runs *first* and short-circuits; otherwise `currentState->handleKey`.
- **Transitions** (`tryTransition` in `Callbacks.cpp`): `R`→Record,
  `Ctrl+R`→Playback, `P`→Pick. Playback/Pick are guarded by `requireWaypoints`.
  `setState` swaps the pointer then calls `onEnter` — the one place onEnter runs.
- **In-mode keys** (per `States.h`): Navigation = arrows move + `<`/`>` altitude;
  Record = `B` stores a waypoint; Playback = `UP`/`DOWN` step waypoints.
- **Resize** (`framebufferSizeCallback`): recomputes `globalView.viewport =
  leftHalf(w,h)`, `playerView.viewport = rightHalf(w,h)`. Pixels, not screen
  coords (HiDPI). The loop only reads viewports — never recomputes them.

## State pattern (`src/state/`)

- `State.h` — base: `onEnter(AppState&)` (default no-op), pure
  `handleKey(AppState&,key,mods)`, `renderGlobalOverlay` / `renderPlayerOverlay`
  (default draw nothing; receive a `Renderer&`, **not** a `Shader`).
- `States.h/.cpp` — `NavigationState`, `RecordState` (overlays path+waypoints on
  global view), `PlaybackState` (private `m_index`), `PickState` (**stub** —
  only `onEnter` clears the path today).
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

## Extension seams (Modes 2–4)

- **Mode 2 (Picking):** `Renderer.h` documents the seam — add a `drawGhost`
  (uniform color/alpha on `m_sceneShader`, drawn through the overlay surface) and
  a private offscreen color-pick pass (`pickAt(x,y)` via a private pick shader +
  FBO + `glReadPixels`). Extract the pick pass into its own `ColorPicker`/
  `PickPass`, not onto `Renderer`. `PickState`/`renderPlayerOverlay` are the
  state-side hooks; add `handleMouseButton` to `State` when needed.
- **Modes 3/4 (Trackers / 2D feature matching):** new `State` subclasses; keep
  OpenCV/CV pipeline code decoupled from rendering.
- **Terrain swap (future menu):** `Renderer::loadTerrain(newMesh)` already exists.

## Gotchas / tripwires

- **GL-handle-outside-Renderer tripwire:** the moment a `State` or `AppState`
  member owns a raw GL handle, invariant #2 breaks (it'd be destroyed after
  `glfwTerminate()`). Then either route it through the `Renderer` facade, or
  introduce a minimal RAII window/context guard *then* (constructed before
  `Renderer`). This is exactly why the `GLContext`/`Application` classes are
  deferred — see memory `glcontext-class-deferred`.


## Deferred (YAGNI — don't re-pitch)

- `Application` class (`docs/application-class-refactor.md`) and `GLContext`
  window guard — both deferred while `Renderer` is the sole GPU owner.
- Regrouping scene/recording fields off `AppState`.

## Cross-references

- `docs/refactor-summary.md` — human-facing overview.
- `docs/renderer-class-refactor.md`, `state-pattern-refactor.md`,
  `application-class-refactor.md` — per-refactor decision notes.
- Memory: `renderer-refactor-status`, `global-appstate-retired`,
  `oop-refactor-direction`, `glcontext-class-deferred`.
