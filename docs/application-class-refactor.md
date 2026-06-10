# Application Class Refactor — Plan (IMPLEMENTED)

Refactor #3 from `renderer-class-refactor.md`. **Implemented** on `legacy-import`: the
unified session loop now lives behind `src/core/Application.{h,cpp}` + a `Window` RAII
wrapper (`src/core/Window.{h,cpp}`), with the **mesh-less `Renderer`** refinement adopted
— so there is **no `std::optional<Renderer>`** and no load-bearing `{ }` scope. The notes
below are kept as the rationale; where they say "deferred" / "would," read "done."

## Problem — what `main.cpp` still juggles by hand

1. **Init-order ritual** in `initGL()` (GLFW → 3.3 core window → GLAD → GL state).
2. **Destruction-order hazard** — `Renderer` must be destroyed while the GL context is alive,
   currently guarded by a load-bearing inner `{ }` scope before `glfwTerminate()`.

> The original third reason — *retiring the global `AppState`* — is **already done** without
> an `Application` class: `appState` is now a local in `main()` (the composition root), passed
> to `initGL(AppState&)` and reached elsewhere only via the GLFW user pointer / `AppState&`
> params. So that motive no longer argues for this class; only the two seams above remain.

> **Which main refactor?** This doc and `main-refactor.md` both answer "main is
> bloated," but they sit on opposite sides of the loop choice — see the
> *"Which refactor? (the fork)"* table in `main-refactor.md`. Short version: on the
> primitive loop, the free-function refactor is the answer; this `Application` class
> activates only if/when the unified loop is adopted. The two compose — the free
> functions become this class's private methods.

## What forces it: the *unified* menu loop

The class only earns its keep once the menu goes from the **primitive** loop
(recreate window + `Renderer` + context per terrain) to a **unified** loop
(create them **once**, swap the mesh in place via `Renderer::loadTerrain`). The
unified loop introduces long-lived GL resources that span many menu picks, plus a
two-signal exit (Escape → back to menu; OS close → quit) — exactly what an
`Application` is for.

## Shape (menu-driven)

```cpp
class Application {
public:
    Application();                       // initWindow() only — no terrain yet
    ~Application();                      // reset Renderer, then tear down GL
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    int run();                           // the menu/session loop
private:
    void initWindow();                   // createWindow + attachContext + configureGLState
    void loadTerrain(const std::string &path);   // read mesh, (emplace|swap) Renderer, seed scene
    void runSession();                   // one terrain's frame loop, until Escape/close

    GLFWwindow             *m_window = nullptr;
    AppState                m_appState;
    AppContext              m_context;   // { &m_appState, &renderer-once-it-exists }
    std::optional<Renderer> m_renderer;  // emplaced after the first terrain (needs live GL)
};
```
`run()` loops `loadTerrain(selectTerrain(...)) → runSession()` while the window is
open; `runSession` exits on `m_appState.returnToMenu` (Escape) or
`glfwWindowShouldClose` (OS close). `main()` shrinks to
`Application app; return app.run();`

## Key points

- **Destruction order becomes unforgettable.** `~Application` does `m_renderer.reset()`
  then `glfwTerminate()` — the brace hazard from `main` is encoded in the type.
- **The `optional` emplace-or-swap is hidden** inside `loadTerrain`; `run()` calls it
  uniformly for every terrain, first and subsequent alike.
- **Two-signal exit** (`returnToMenu` vs `windowShouldClose`) lives in `runSession`/`run`,
  not cluttering `main`.
- **The menu simplifies the class** vs. the older single-terrain `loadScene` sketch: every
  `run()` iteration picks a terrain, so terrain loading is symmetric inside the loop.

## Refinement: a mesh-less `Renderer` removes the `optional` entirely

Today `Renderer(const Mesh&)` conflates two jobs — compile the shader programs (needs
only a live context) and upload a *specific* terrain (mutable content). That marriage is
what forces the `std::optional` and the first-vs-rest asymmetry above.

Split them: a mesh-less `Renderer()` that only builds the pipeline, with `loadTerrain()`
as the **sole** path for terrain data. Then pair it with a `Window`/`GLContext` RAII member
declared **before** the `Renderer`:

```cpp
class Application {
    Window   m_window;     // ctor: glfwInit + create + GLAD → context is live
    AppState m_appState;
    Renderer m_renderer;   // ctor: compile shaders — context already live (mesh-less)
    AppContext m_context;
    // ...
};
```

Member-init order does the work: `m_window` constructs first (context live), so the plain
`m_renderer` compiles safely; reverse-order destruction runs `~Renderer` (delete GL handles)
**before** `~Window` (`glfwTerminate`). Result: **no `optional`, no manual `reset()`**, and
every terrain is just a `loadTerrain` call. This is the companion to the deferred `GLContext`
guard — the mesh-less ctor is the half that makes that guard pay off.

Cost (honest): a mesh-bearing ctor guarantees "a `Renderer` always has a terrain"; the
mesh-less one makes "no terrain loaded yet" representable, so render paths must be
guaranteed-after-`loadTerrain` (true in the session loop) or guard against an empty terrain.

## Why we're on the primitive loop anyway (deliberate)

The primitive loop was chosen **on purpose** so exactly these problems would surface — the
`optional<Renderer>`, the destruction-order brace, the two-signal exit. Feeling that friction
firsthand is what makes the unified loop + `Application` + mesh-less `Renderer` legible as the
*answer* to a real need, rather than abstraction adopted on faith. None of this is justified on
the primitive loop itself, where `Renderer renderer(mesh);` is a clean stack object.

## What it does NOT do

Not own picking / CV pipelines (Mode-2). Not a game-engine/framework abstraction — one window,
one app, no virtuals.

## Urgency — YAGNI: defer

With the global already gone, the remaining wins are a pure coordination seam (init/teardown
centralization, menu session loop). No lifetime bomb pressures it: `State` subclasses own no GL
resources. PICK (Mode 2) shipped **without** forcing the class — its mouse callback reaches the
renderer through `AppContext`, and the pick pass draws into the back buffer (no window-owned FBO
yet). So the live trigger is the **unified menu loop** (see above): build the class when we adopt
it, so the design is driven by a concrete need rather than guessed at now.

## Files / verification (when implemented)

- **New:** `src/core/Application.{h,cpp}`. **Edit:** `src/main.cpp` (shrink to construct + run;
  remove `initGL`). `initGL()`'s body → `Application::initWindow()`.
- `make` clean; `./bin/drone_sim` renders the same two-viewport scene; modes behave as before;
  clean exit with no GL errors (confirms destruction order).
