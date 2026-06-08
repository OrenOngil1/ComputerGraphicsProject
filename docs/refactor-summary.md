# OOP Refactor — Summary

A short, human-readable overview of the architecture refactor on branch `oop`.
For the reasoning behind each decision, see the linked decision notes.

## In one sentence

`main.cpp` used to be a procedural script that owned everything and steered the
program with `switch (mode)` statements; it is now a thin **composition root**
over a handful of small, single-responsibility types.

## What changed

| Area | Before | After |
|------|--------|-------|
| **GPU resources** | `Shader` + terrain buffers lived in a bare `{ }` scope in `main()` with a load-bearing comment | A **`Renderer`** owns them; lifetime is a property of the object. One `render*View` call per viewport. |
| **Modes** | Behaviour split across two `switch (mode)` blocks (input + rendering) | One **`State`** subclass per mode (Navigation / Record / Playback / Pick). Adding a mode = adding a class, no `switch` edits. |
| **Camera vs. screen** | One struct mixed eye-pose with viewport rectangle | Split into **`Camera`** (eye/lens) + **`Viewport`** (screen rect), bundled into a **`View`**. |
| **Layout** | Viewport math recomputed in the render loop | Recomputed **once per resize** in the framebuffer callback; the loop just reads it. |
| **Global state** | A global `AppState` | A **local** `AppState` in `main()`, reached elsewhere via the GLFW window user pointer. |

## Directory layout

```
src/
  main.cpp        composition root: init GL, build scene, run the frame loop
  core/           AppState, Camera/Viewport/View, Scene (Mesh/Vertex), Waypoint
  render/         Renderer — owns the scene shader + terrain GPU buffers
  state/          State interface + the concrete modes
  input/          Callbacks (keys, resize) + Movement
  loader/         TerrainLoader — reads a DEM image into a Mesh
  engine/         thin OpenGL 3.3 wrappers (Shader, VertexArray/Buffer, ...)
```

## Key design rules (so the structure stays clean)

- **`Renderer` is the single owner of GPU resources.** A mode's overlay draws
  *through* the Renderer (it receives a `Renderer&`, never the raw shader).
- **No mode `id()`/enum tag** — "which modes exist" is "which `State` subclasses
  exist." Transitions live in one place (`Callbacks.cpp`), not inside states.
- **A `Camera` is read-only render input;** a `Viewport` is pure layout.

## Deferred (intentionally, YAGNI)

- An **`Application` class** wrapping GL init/teardown — not needed while
  `Renderer` is the sole GPU owner. See `application-class-refactor.md`.
- A **`GLContext`/window guard** — same reasoning; revisit only when a GL handle
  is first owned outside `Renderer`.
- Regrouping scene/recording data on `AppState`.

## Detailed decision notes

- `renderer-class-refactor.md` — why a concrete (non-virtual) Renderer.
- `state-pattern-refactor.md` — the State pattern as built.
- `application-class-refactor.md` — the deferred Application class.
