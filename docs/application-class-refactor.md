# Application Class Refactor — Plan (DEFERRED)

Refactor #3 from `renderer-class-refactor.md`. **Not implemented — deferred (YAGNI).**
Kept ready for the moment a feature forces it.

## Problem — what `main.cpp` still juggles by hand

1. **Init-order ritual** in `initGL()` (GLFW → 3.3 core window → GLAD → GL state).
2. **Destruction-order hazard** — `Renderer` must be destroyed while the GL context is alive,
   currently guarded by a load-bearing inner `{ }` scope before `glfwTerminate()`.

> The original third reason — *retiring the global `AppState`* — is **already done** without
> an `Application` class: `appState` is now a local in `main()` (the composition root), passed
> to `initGL(AppState&)` and reached elsewhere only via the GLFW user pointer / `AppState&`
> params. So that motive no longer argues for this class; only the two seams above remain.

## Shape

```cpp
class Application {
public:
    Application();                      // initWindow() + loadScene()
    ~Application();                     // reset Renderer, then tear down GL
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    int run();                          // the render loop
private:
    bool initWindow();                  // initGL()'s body moves here
    void loadScene();                   // readTerrain, place cameras, seed NavigationState, emplace Renderer
    GLFWwindow             *m_window = nullptr;
    AppState                m_appState;
    std::optional<Renderer> m_renderer; // emplaced once GL is live
};
```
`main()` shrinks to `Application app; return app.run();`

## Key points

- **Deferred `Renderer` (`std::optional`).** Its ctor needs a live GL context (compiles
  shader, uploads terrain), so it is `emplace`d in `loadScene()`, after `initWindow()`.
- **Destruction order.** `~Application` explicitly `m_renderer.reset()` before
  `glfwDestroyWindow`/`glfwTerminate` — centralizes today's brace hazard in one place.
- **State seeding moves in.** `currentState = make_unique<NavigationState>()` moves into
  `loadScene()`; `Application` transitively owns the state machine via `m_appState`.
- **Menu synergy.** A long-lived `Application` + `Renderer::loadTerrain()` is the right home
  for the menu's DEM swap (create the context once, swap terrain) — not a `while(true)` around
  `initGL`. The menu is the feature most likely to force this class into existence.

## What it does NOT do

Not own picking / CV pipelines (Mode-2). Not a game-engine/framework abstraction — one window,
one app, no virtuals.

## Urgency — YAGNI: defer

With the global already gone, the remaining wins are a pure coordination seam (init/teardown
centralization, menu session loop). No lifetime bomb pressures it: `State` subclasses own no GL
resources. Build it when the **menu** or **PICK** forces its shape (PICK needs *both* the window
and renderer for mouse callbacks + an offscreen FBO), so the design is driven by a concrete need.

## Files / verification (when implemented)

- **New:** `src/core/Application.{h,cpp}`. **Edit:** `src/main.cpp` (shrink to construct + run;
  remove `initGL`). `initGL()`'s body → `Application::initWindow()`.
- `make` clean; `./bin/drone_sim` renders the same two-viewport scene; modes behave as before;
  clean exit with no GL errors (confirms destruction order).
