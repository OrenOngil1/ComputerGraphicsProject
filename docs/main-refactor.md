# main.cpp decomposition (proposal — discussion only, not yet implemented)

Status: **proposed / awaiting decision** — recorded for review.

## The problem

After re-introducing the outer menu loop, `src/main.cpp` (165 lines) braids three
unrelated concerns into one file:

1. **Platform bootstrap** — `initGL`: glfwInit, context hints, window creation,
   GLAD load, callback wiring, the initial viewport seed, persistent GL state.
2. **Scene configuration** — the two ~18-line camera literals (lines 104–121),
   which are *data*, not control flow.
3. **Orchestration** — the outer terrain loop, the inner render loop, the
   Renderer-scoping block.

The two loops are genuine orchestration and belong in `main`. The rest doesn't:
the camera literals bury main's shape, and `initGL` mixes "any OpenGL app needs
this" with "our app needs this" so you can't tell the two apart line-by-line.

This isn't a mess — the pieces are sound and well-commented. It's muddy
separation-of-concerns at the one file every reader opens first.

## Proposed shape: five free functions

Keep it small. No `Application`/`Engine` class — that would trade 165 readable
lines for member/init/run/shutdown ceremony and more indirection, which is
over-engineering at this scale.

| Helper | Absorbs from today's `main`/`initGL` | Concern |
| --- | --- | --- |
| `GLFWwindow* createWindow()` | glfwInit, hints, createWindow, makeContextCurrent, gladLoad, version print | platform bootstrap (no app knowledge) |
| `void configureGLState()` | clearColor / clearDepth / depthFunc / depthTest / blendFunc / blend | render-pipeline policy |
| `void attachContext(window, ctx)` | user-pointer, the 3 callbacks, the initial viewport seed | app-specific wiring (the app half of `initGL`) |
| `void configureViews(appState)` | the two camera literals | scene data, not control flow |
| `void runFrameLoop(window, appState, renderer)` | the dt / tick / clear / render / swap / poll loop | the inner loop, named to flatten nesting |

`initGL` dissolves: its generic half becomes `createWindow`, its app half becomes
`attachContext` + `configureGLState`.

### Resulting main()

    int main() {
        while (true) {
            AppState appState;
            appState.mesh = readTerrain(selectTerrain("assets/terrains/"));
            appState.terrainSize = std::max(appState.mesh.width, appState.mesh.height);

            AppContext context{ &appState, nullptr };
            GLFWwindow *window = createWindow();
            if (!window) return -1;
            attachContext(window, context);
            configureGLState();

            configureViews(appState);
            appState.currentState = std::make_unique<NavigationState>();

            {
                Renderer renderer(appState.mesh);
                context.renderer = &renderer;
                runFrameLoop(window, appState, renderer);
            }
            glfwTerminate();
        }
    }

Reads top-to-bottom as: build app-state → bring up window+GL → configure scene → run.

## Open decision: where the helpers live

**Option A (lean):** all five as `static` functions atop `main.cpp`.
- Pros: zero new files, lowest ceremony, fits project scale; the entry point stays self-contained.
- Cons: the reusable platform half (`createWindow`/`configureGLState`) still physically lives in main.

**Option B:** move `createWindow` + `configureGLState` into `src/core/Bootstrap.{h,cpp}`;
keep the three app-aware helpers in main.
- Pros: honest platform/app split; the platform half becomes independently readable.
- Cons: a new header+source for ~60 lines; only clearly pays off if a 2nd context or a
  headless path ever appears (out of scope).

Recommendation: **A now, B as the upgrade path** if platform code ever grows.

## Verification (when implemented)

Pure extraction, no behavior change. `make` links cleanly; `./bin/drone_sim` behaves
identically — menu, WASD/arrow flight, RECORD/PLAYBACK/PICK, Escape → menu, Exit → quit.
