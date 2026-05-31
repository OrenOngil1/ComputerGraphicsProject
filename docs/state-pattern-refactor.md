# State Pattern Refactor — As-Built

Refactor #4 from `docs/renderer-class-refactor.md`. **Implemented** on branch `oop`.
This documents what was actually built (the design evolved during implementation).

## Problem (recap)

"What a mode does" was spread across two `switch (mode)` statements — input dispatch in
`Callbacks.cpp` and overlay rendering in `Renderer.cpp`. Each new mode meant editing both
(shotgun surgery). Replaced with one polymorphic `State` per mode.

## The interface — `src/state/State.h`

```cpp
class State {
public:
    virtual ~State() = default;
    virtual void onEnter(AppState &) {}                       // entry action (post-swap)
    virtual void handleKey(AppState &, int key, int mods) = 0;
    virtual void renderGlobalOverlay(const AppState &, Shader &, const glm::mat4 &) const {}
    virtual void renderPlayerOverlay(const AppState &, Shader &, const glm::mat4 &) const {}
};
```

Deliberately **no `Mode`/`id()` tag** — a per-class enum constant is a type label that
invites `switch (state->id())`, reintroducing exactly what the pattern removes. "Which
modes exist" is answered by "which `State` subclasses exist." Pure interface: depends only
on forward declarations of `AppState`/`Shader` + glm.

## Concrete states — `src/state/States.{h,cpp}`

Grouped in one `.h`/`.cpp`: each is tiny, they form one cohesive set, they co-change, and
only `Callbacks.cpp` constructs them. The base `State.h` stays **separate** so high-level
code (`Renderer`) depends on the abstraction, not the concretes (Dependency Inversion).

- **NavigationState** — `handleKey` → `handleMovement`. No overlay, no `onEnter`.
- **RecordState** — `onEnter` clears `pathPoints` + `cameraRecords`; `handleKey` moves +
  appends a path point on move + `B` appends a waypoint; overlay draws path + records.
- **PlaybackState** — `onEnter` snaps camera to `records[0]` (`m_index = 0`); `handleKey`
  steps `m_index` (advance-then-apply) and snaps the camera; overlay draws path + records.
  Owns a local `size_t m_index`.
- **PickState** (stub) — `onEnter` clears `pathPoints`; `handleKey` empty.

Mode logic was **folded in**: deleted `RecordInput`, `PlaybackInput`, `Recording`. Kept
`Movement` free — `handleMovement` is shared by Navigation *and* Record, so folding it into
one would break the other. (That's the line: fold per-mode dispatchers, keep shared helpers.)

## Ownership — `AppState` holds the current state

`std::unique_ptr<State> currentState` replaced `Mode mode`. Forward-declared in
`AppState.h`; out-of-line `~AppState()` in `AppState.cpp` (a `unique_ptr` to an incomplete
type needs the destructor where `State` is complete). Seeded to `NavigationState` in
`main.cpp` before the loop. The global `appState` is retained (Application deferred).

## Data ownership — mode-local data lives in the state

Ownership follows **lifetime**, not usage. Mode-local scratch lives in the state (born/dies
with it); shared data stays in `AppState`.

- **Mode-local:** `PlaybackState::m_index` (was `AppState.playbackIndex`); and, for PICK
  later, `pickedPoints` / `computedCamera`.
- **Shared (`AppState`):** `mesh`, `terrainSize`, cameras, `pathPoints`, `cameraRecords`.

This is why oren's scattered resets collapse: `pickedPoints.clear()` /
`computedCamera = null` / `playbackIndex = 0` become **automatic** (mode-local
construction/destruction); only the shared `pathPoints` / `cameraRecords` need explicit
`onEnter` clears.

## Transitions — guarded, in the context (not on the interface)

In `Callbacks.cpp`:
- `setState(appState, next)` — swaps `currentState`, then calls `next->onEnter(appState)`.
- `requireRecords(appState)` — PLAYBACK and PICK require ≥1 recorded camera; checked
  **before** the swap (a precondition is a property of the *transition*; `onEnter` runs
  after the swap and is too late to refuse).
- `tryTransition(appState, key, mods)` — global hotkeys `R` / `Ctrl+R` / `P`; returns true
  if handled (performed **or** refused).
- `keyCallback`: `if (tryTransition(...)) return; else currentState->handleKey(...)`.

No `changeMode()` on the `State` interface — transitions are global app commands, not
per-mode behavior. Running them before/separately from `handleKey` also dodges the
self-deletion hazard (a state reassigning the pointer that owns it mid-method).

**The split:** guard (pre-swap) = "may I enter?"; `onEnter` (post-swap) = "set up now that
I'm current." No `onExit` — no customer, and a state's destructor covers its own teardown.

## Clearing matrix (matches oren)

| Entering | `pathPoints` | `cameraRecords` | camera |
|----------|--------------|-----------------|--------|
| RECORD   | clear        | clear           | — |
| PLAYBACK | keep         | keep            | snap to `records[0]` |
| PICK     | clear        | keep            | (seed from a record — with full PICK) |

## Highlight — by camera position (oren), state-independent

`renderCameraRecords(records, playerCamera.position)` greens the record whose position
matches the camera, red otherwise. The marker tracks the **camera** in any mode; `m_index`
is never used for the highlight, so they cannot desync. The exact float compare is safe —
PLAYBACK snaps the camera to an *exact copy* of a record's position. `m_index` is purely the
navigation cursor.

## Renderer

`Renderer::renderGlobalView` / `renderPlayerView` each draw the terrain (via a shared
private `renderScene`) and then delegate the overlay to `currentState` (`renderGlobalOverlay`
/ `renderPlayerOverlay` respectively); `Renderer` no longer knows about `Mode`, and there is
no overlay flag/enum that could be mismatched with the view. The old free
`renderGlobalOverlay` / `renderPlayerOverlay` free functions (with `switch (mode)`) were
deleted.

## Known follow-ups

- **PLAYBACK entry-snap** is a deliberate deviation from oren (which snapped on the first
  key); chosen for cleaner UX.
- **PICK is a stub.** Its real implementation adds: `handleMouseButton` (a mouse hook on
  `State`), the offscreen pick pass (Renderer `m_pickShader` seam, fixed-function → core
  reimplementation), the ghost overlay (`renderPlayerOverlay`, which needs renderer access
  to the terrain), and seeding the camera from a record in `onEnter` (oren: a random
  record). The guarded + parameterized entry it needs is already in place
  (`requireRecords` guard + `onEnter`).

## Verification

`make` clean; `R` records (`B` adds waypoints, green when the camera is on one); `Ctrl+R`
requires records, snaps to record 0, `UP`/`DOWN` step; `P` requires records, clears the
path; navigation arrows / `<` `>` in NONE. With no records, PLAYBACK/PICK are refused with
a console message.
