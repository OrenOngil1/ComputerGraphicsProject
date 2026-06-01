# Renderer Class Refactor — Decision Note

> **Status:** Implemented on branch `oop`. Companion docs: `state-pattern-refactor.md`
> (#4, done) and `application-class-refactor.md` (#3, deferred — YAGNI).

## Original question

Worth adding a Renderer class that abstracts the per-viewport loop (viewport → MVP → terrain →
overlay), or is it over-engineering? If worth it, where, and why?

## Verdict: worth it — the *concrete* version, not a polymorphic hierarchy

The pre-existing free-function style (`uploadTerrain`, `computeViewProjection`, `drawTerrain`,
…) was fine. The class solves **ownership and duplication**, not function style:

1. **Unowned GPU-resource lifetime.** `Shader` + `TerrainGpu` used to live in a bare `{ }`
   scope in `main()` with a load-bearing comment ("destroying after `glfwTerminate()` crashes").
   A `Renderer` that *owns* them makes the lifetime a property of an object, not a comment.
2. **Duplicated per-viewport block.** The same four steps ran twice in `main()`; collapsed into
   one call site per view.
3. **No home for future overlays.** Modes 2–4 (picking spheres, trackers, ghost) each add scene
   geometry; without a Renderer they bolt onto `main()`'s loop.

**Where it stops being worth it:** a `Renderer` *interface* with virtual subclasses
(`GlobalRenderer`, …) is over-engineering — there's one way to draw the scene; only the *overlay*
varies, and that varies by mode, not renderer type. So: **one concrete class, no virtuals.**

## As-built shape (`src/render/Renderer.{h,cpp}`)

```cpp
class Renderer {
public:
    explicit Renderer(const Mesh &terrain);   // owns Shader + TerrainGpu (RAII)
    void clear() const;
    void loadTerrain(const Mesh &terrain);    // menu DEM swap (reuses the shader)
    void renderGlobalView(const Camera &, const Viewport &, const AppState &);
    void renderPlayerView(const Camera &, const Viewport &, const AppState &);
    // overlay drawing surface — modes draw *through* these, shader never leaves Renderer:
    void drawPath(const std::vector<glm::vec3> &, const glm::mat4 &);
    void drawWaypoints(const std::vector<Waypoint> &, const glm::vec3 &camPos, const glm::mat4 &);
private:
    glm::mat4 renderScene(const Camera &, const Viewport &);  // viewport + MVP + terrain
    Shader     m_sceneShader;
    TerrainGpu m_terrain;
};
```
- **Two view methods, not a flag.** `renderGlobalView`/`renderPlayerView` each draw the terrain
  (shared private `renderScene`) then delegate the overlay to `currentState` — the view can never
  be mismatched with its overlay, and `Renderer` no longer knows `Mode`.
- `uploadTerrain`/`computeViewProjection`/`drawTerrain` are free helpers in the `.cpp`;
  `leftHalf`/`rightHalf`/`setupViewport` stay free (pure window geometry, no renderer state).

## Where it fits among the five refactors

1. **Camera/Viewport split** — ✅ done.
2. **Renderer class (this)** — ✅ done. RAII over `Shader`+`TerrainGpu`; collapsed the duplicated
   block; added `loadTerrain()`.
3. **Application class** — ⏸️ deferred (YAGNI). Its one concrete win, retiring the global, was
   since achieved another way; the rest is a coordination seam. See `application-class-refactor.md`.
4. **State pattern** — ✅ done. The two `switch (mode)` statements moved into `State` objects;
   `Mode mode` → `unique_ptr<State> currentState`. See `state-pattern-refactor.md`.
5. **AppState decomposition** — ⚠️ partial. The global is **retired** (`appState` is now a `main()`
   local), but the struct still bundles scene (`mesh`, `terrainSize`), two cameras, recording
   (`pathPoints`, `waypoints`) and mode (`currentState`) flat. A cohesion regroup into
   `Scene`/`Recording` sub-structs is the remaining tidy — deferred, not a god *object* (no behavior).

**Why this order:** each step compiles and runs alone (collaborative, incremental pace). Renderer
first (dependency-free, immediate lifetime-safety + dedup payoff); Application deferred; State built
on Renderer alone — it needed only somewhere for `currentState` to live (`AppState`, for now).

## What this is NOT

Not a render-graph / material / pipeline abstraction or virtual dispatch — unjustified at this
scale. Renderer must not own the window or poll events (`Application`'s job) nor know `Mode`
semantics (the State's job). Its remit: "given a camera, a viewport, and what to draw, paint one view."

## Verification

`make` clean; `./bin/drone_sim` renders the same two-viewport scene; RECORD/PLAYBACK overlays still
appear in the left view. Pure structural refactor — the visual output is the test.
