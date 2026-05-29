# Renderer Class Refactor — Decision Note

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

1. **Camera/Viewport split** — done.
2. **Renderer class (this one)** — do it *next*, before `Application`. It is the most
   self-contained win: it owns GPU resources and removes a documented crash hazard with
   no dependency on the State or AppState refactors. `Application` will want to *hold* a
   `Renderer`, so the Renderer should exist first.
3. **Application class** — owns the `GLFWwindow`, the `Renderer`, and the `AppState`, and
   runs the main loop. This is where `main()`'s body migrates. Depends on Renderer.
4. **State pattern for Mode + AppState decomposition** — do these together. **This is the
   one design coupling to decide deliberately:** overlays are mode-specific, so resist
   the temptation to make `Renderer` `switch` on `Mode`. Instead, the current
   `renderGlobalOverlay`'s `switch (appState.mode)` should eventually move *into the State
   objects* — each `State` knows how to draw its own overlay, and `Renderer::renderView`
   either calls `state.renderOverlay(...)` or takes a small overlay callback. This keeps
   Renderer closed to modification as Modes 3/4 are added (Open/Closed principle).

**Why this order:** each step compiles and runs on its own, matching the project's
collaborative, incremental pace. Renderer first because it is dependency-free and pays
off immediately (lifetime safety + dedup); State last because it is the broadest change
and benefits from Renderer + Application already being in place to plug into.

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
