# Application Class Refactor — Plan (DEFERRED)

Refactor #3 from `docs/renderer-class-refactor.md`. **Not implemented — deliberately
deferred (YAGNI).** See "Urgency" below. This plan is kept ready for the moment a feature
forces it.

## Problem

`src/main.cpp` still juggles three things by hand:
1. A **global** `AppState appState;` — unbounded access (any TU can mutate it) and static
   storage duration (destroyed *after* `main()` returns, i.e. after `glfwTerminate()`).
2. The **init-order ritual** in `initGL()` (GLFW → 3.3 core window → GLAD → GL state).
3. The **destruction-order hazard** — the `Renderer` must be destroyed while the GL context
   is alive, currently guarded by a load-bearing inner `{ }` scope before `glfwTerminate()`.

## Shape

```cpp
class Application {
public:
    Application();                                  // initWindow() + loadScene()
    ~Application();                                 // reset Renderer, then tear down GL
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    int run();                                       // the render loop
private:
    bool initWindow();   // glfwInit -> 3.3 core window -> GLAD -> GL state -> user ptr -> callbacks
    void loadScene();    // readTerrain, place cameras, seed NavigationState, emplace Renderer
    GLFWwindow             *m_window = nullptr;
    AppState                m_appState;
    std::optional<Renderer> m_renderer;              // emplaced once GL is live
};
```

`main()` shrinks to: `Application app; return app.run();`

## Key points (updated to the current codebase)

- **Retires the global.** `m_appState` becomes a member;
  `glfwSetWindowUserPointer(m_window, &m_appState)` keeps the input callbacks working
  unchanged — they already fetch `AppState` through the user pointer, and nothing outside
  `main.cpp` references the global.
- **Deferred `Renderer` (`std::optional`).** The `Renderer` constructor needs a live GL
  context (it compiles the shader and uploads terrain), which only exists *after*
  `initWindow()`. So it is `emplace`d in `loadScene()`, not built in the init list.
- **Destruction order (chosen approach):** `~Application` explicitly `m_renderer.reset()`
  before `glfwDestroyWindow` / `glfwTerminate`. Member destructors run *after* the body, so
  the explicit reset is required. This centralizes the hazard in one place instead of the
  brace in `main()`.
- **State seeding moves in.** `main.cpp` currently seeds
  `appState.currentState = make_unique<NavigationState>()` before the loop; that moves into
  `loadScene()`. `Application` owns `AppState`, which owns `currentState`, so the state
  machine is transitively owned — no separate handling needed.
- **Menu synergy.** A long-lived `Application` + `Renderer::loadTerrain()` is the right home
  for the menu's DEM swap: create the context **once**, swap terrain via `loadTerrain()` —
  not the legacy `while(true)`-around-`initGL`. The menu is the feature most likely to force
  `Application` into existence.

## What it does NOT do

- Not own picking / CV pipelines (Mode-2 concerns).
- Not a game-engine / framework abstraction. One window, one app — no virtuals, no plugins.

## Urgency — YAGNI: defer

**Not urgent.** The one concrete here-and-now benefit is retiring the global; everything
else (a coordination seam) is anticipatory. The State refactor did **not** add urgency:
`AppState` now holds `unique_ptr<State> currentState`, but `State` subclasses own **no GL
resources**, so the global's static destruction (after `glfwTerminate`) is still safe —
there is no lifetime bomb pressuring us.

**Recommendation:** keep deferring until the **menu** or **PICK** forces it, so its shape is
driven by a concrete need (the menu's session loop; or PICK's offscreen FBO + mouse
callbacks needing *both* the window and the renderer) rather than a guess. Build them
together with `Application` when that moment comes.

## Files (when implemented)

- **New:** `src/core/Application.{h,cpp}`.
- **Edit:** `src/main.cpp` (shrink to construct + run; remove the global `AppState` and
  `initGL`), `src/core/AppState.h` (drop `extern AppState appState;`). `initGL()`'s body
  moves into `Application::initWindow()`.

## Verification (when implemented)

`make` clean; `./bin/drone_sim` renders the same two-viewport scene; modes behave as before;
clean exit with no GL errors / crash (confirms the destruction order is intact).
