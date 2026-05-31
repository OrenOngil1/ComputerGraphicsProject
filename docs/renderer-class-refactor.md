# Renderer Class Refactor — Decision Note

> **Status (updated):** Implemented on branch `oop`. The decision below stands; the build
> added `loadTerrain()` (a menu seam), and `renderGlobalView` / `renderPlayerView` (two
> intent-revealing methods, not a flag) delegate the overlay to the active `State` — the #4
> prediction came true. Companion
> docs: `state-pattern-refactor.md` (#4, done) and `application-class-refactor.md`
> (#3, deferred — YAGNI).

## The Prompt (original question)

> In another session, you recommended the following changes to the codebase:
> separating camera and viewport, decomposing AppState, implementing the State
> pattern, and creating an Application class. I have considered a 5th addition: a
> renderer class that abstracts away the different loop of computing viewport, mvp
> and rendering the terrain and the appropriate overlay. What do you think: does it
> worth it, the benefit outweighing the time spent on the refactor, or is it
> over-engineering? If it is worth it, where should I add this refactor? And for
> all, explain WHY.

---

## Verdict: Worth it — but only the *concrete* version, not a polymorphic hierarchy

Right now `src/render/Renderer.{h,cpp}` is a **module of free functions** over plain
data (`uploadTerrain`, `computeViewProjection`, `renderTerrain`, `renderGlobalOverlay`,
`renderPlayerOverlay`). That functional style is genuinely good and is *not* the
problem. The problem the class solves is **ownership and duplication**, visible in
`main.cpp:106-137`:

1. **Unowned GPU resource lifetime.** `Shader sceneShader` and `TerrainGpu terrainGpu`
   live in a bare `{ }` scope in `main()` with a load-bearing comment warning that
   destroying them after `glfwTerminate()` crashes. That hazard is real but invisible —
   nothing in the type system enforces it. A `Renderer` object that *owns* the shader +
   terrain GPU buffers makes the lifetime a property of an object, not a comment.

2. **Duplicated per-viewport block.** Lines 118-124 and 126-132 are the same four steps
   twice (setupViewport → computeViewProjection → renderTerrain → render*Overlay). A
   `renderView(camera, viewport, appState)` method collapses both into one call site.

3. **No home for future overlays.** Modes 2-4 (picking spheres, trackers, ghost camera)
   each add scene geometry. Without a Renderer they get bolted onto `main()`'s loop,
   which is exactly the "messy state separation" the project wants to avoid.

**Where it stops being worth it:** a `Renderer` *interface* with virtual methods and
subclasses (`GlobalRenderer`, `PlayerRenderer`, …) would be over-engineering for a
two-viewport app. There is one way to draw the scene; the only variation is *which
overlay*, and that varies by Mode, not by renderer type. So: one concrete class, no
virtuals.

## Recommended shape

A single concrete class in `src/render/Renderer.{h,cpp}`, absorbing the existing free
functions as private helpers / implementation detail:

```cpp
class Renderer {
public:
    Renderer(const Mesh& terrain);          // owns Shader + TerrainGpu (RAII)
    void clear() const;                      // glClear
    void renderView(const Camera& camera,
                    const Viewport& viewport,
                    const AppState& appState); // setup + MVP + terrain + overlay
private:
    Shader  m_shader;
    TerrainGpu m_terrain;
};
```

- `uploadTerrain`, `computeViewProjection`, `renderTerrain` become implementation
  details behind the class (keep them as free functions in the .cpp, or private methods).
- `leftHalf` / `rightHalf` / `setupViewport` stay free functions — they are pure
  window-geometry helpers with no dependency on renderer state.
- `main()`'s loop shrinks to: `renderer.clear(); renderer.renderView(globalCam, left, st);
  renderer.renderView(playerCam, right, st);`

## Where it fits among the other four refactors — and why

Ordering matters because these refactors depend on each other:

1. **Camera/Viewport split** — ✅ done.
2. **Renderer class (this one)** — ✅ done. Owns `Shader` + `TerrainGpu` (RAII); collapsed
   the duplicated per-viewport block into `renderView`; added `loadTerrain()` for the menu.
3. **Application class** — ⏸️ deferred (YAGNI). Its dependency on Renderer is satisfied, but
   it isn't urgent: its only concrete win is retiring the global `appState`, and `State`
   objects own no GL resources, so the global's static destruction stays safe. Build it
   when the menu or PICK forces it. See `application-class-refactor.md`.
4. **State pattern for Mode + AppState decomposition** — ⚠️ *partly* done.
   - **State pattern: done.** The `switch (appState.mode)` moved into the State objects (each
     draws its own overlay, `Renderer::renderGlobalView` / `renderPlayerView` call
     `state.render*Overlay(...)`, Renderer no longer knows `Mode`); `Mode mode` became
     `unique_ptr<State> currentState` (the enum was dropped, not kept). See
     `state-pattern-refactor.md`.
   - **AppState decomposition: NOT done.** `AppState` still bundles scene (`mesh`,
     `terrainSize`), view (two cameras), recording (`pathPoints`, `cameraRecords`) and mode
     (`currentState`) in one context object passed by reference everywhere. It remains a god
     object — the name is accurate, not aspirational.

**Why this order (as it played out):** each step compiles and runs on its own, matching the
project's collaborative, incremental pace. Renderer went first (dependency-free, immediate
payoff: lifetime safety + dedup). Application was then *deferred* rather than built, so the
State pattern was done on top of Renderer alone — it turned out not to need Application, only
somewhere for `currentState` to live (it went on `AppState` for now, and migrates into
`Application` if/when that lands). State last because it was the broadest change.

## What this is NOT

- Not a render-graph, not a material/pipeline abstraction, not virtual dispatch. Those
  are real patterns but unjustified at this scale — adding them now is the over-engineering
  failure mode.
- Renderer should not own the window or poll events (that is `Application`'s job) and
  should not know about `Mode` semantics (that is the State's job). Keep it to: "given a
  camera, a viewport, and what to draw, paint one view."

## Verification

After implementing: `make` must build clean, `./bin/drone_sim` must render the same
two-viewport scene as today, and RECORD/PLAYBACK overlays must still appear in the left
view. No behavior change — this is a pure structural refactor, so the visual output is
the test.
