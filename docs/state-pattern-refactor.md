# State Pattern Refactor — As-Built

Refactor #4 from `renderer-class-refactor.md`. **Implemented** on branch `oop`. Documents what
was built (the design evolved during implementation).

## Problem

"What a mode does" was split across two `switch (mode)` statements — input in `Callbacks.cpp`,
overlay rendering in `Renderer.cpp`. Each new mode meant editing both (shotgun surgery). Replaced
with one polymorphic `State` per mode.

## The interface — `src/state/State.h`

```cpp
class State {
public:
    virtual ~State() = default;
    virtual void onEnter(AppState &) {}                                  // entry action (post-swap)
    virtual void handleKey(AppState &, int key, int mods) = 0;
    virtual void renderGlobalOverlay(const AppState &, Renderer &, const glm::mat4 &) const {}
    virtual void renderPlayerOverlay(const AppState &, Renderer &, const glm::mat4 &) const {}
};
```

- **No `Mode`/`id()` tag** — a per-class enum invites `switch (state->id())`, the very thing the
  pattern removes. "Which modes exist" = "which `State` subclasses exist."
- Overlays take **`Renderer &`, not `Shader &`**: the mode draws *through* the Renderer
  (`drawPath`/`drawWaypoints`), so `m_sceneShader` never leaves its owner. Depends only on forward
  declarations of `AppState`/`Renderer` + glm.

## Concrete states — `src/state/States.{h,cpp}`

Grouped in one `.h`/`.cpp` (each tiny, one cohesive set, co-change, only `Callbacks.cpp`
constructs them). `State.h` stays separate so `Renderer` depends on the abstraction (DIP).

- **NavigationState** — `handleKey` → `handleMovement`. No overlay.
- **RecordState** — `onEnter` clears `pathPoints`+`waypoints`; `handleKey` moves, appends a path
  point on move, `B` appends a waypoint; overlay draws path + waypoints.
- **PlaybackState** — `onEnter` snaps camera to `waypoints[0]` (`m_index = 0`); `handleKey` steps
  `m_index` and snaps; overlay draws path + waypoints. Owns a local `size_t m_index`.
- **PickState** (stub) — `onEnter` clears `pathPoints`; `handleKey` empty.

Folded in (deleted `RecordInput`/`PlaybackInput`/`Recording`). Kept `Movement` free —
`handleMovement` is shared by Navigation *and* Record. (The line: fold per-mode dispatchers, keep
shared helpers.)

## Ownership — by lifetime, not usage

`std::unique_ptr<State> currentState` replaced `Mode mode`. Forward-declared in `AppState.h`;
out-of-line `~AppState()` in `AppState.cpp` (a `unique_ptr` to an incomplete type needs its
destructor where `State` is complete). Seeded to `NavigationState` in `main.cpp`. `appState` is a
`main()` local (the global was retired; Application still deferred).

- **Mode-local** (lives in the state): `PlaybackState::m_index`; later PICK's
  `pickedPoints`/`computedCamera`. Their resets become **automatic** via construction/destruction.
- **Shared** (`AppState`): `mesh`, `terrainSize`, cameras, `pathPoints`, `waypoints` — only these
  need explicit `onEnter` clears.

## Transitions — guarded, in the context (not on the interface) — `Callbacks.cpp`

- `setState(appState, next)` — swaps `currentState`, then `next->onEnter(appState)`.
- `requireWaypoints(appState)` — PLAYBACK and PICK require ≥1 waypoint; checked **before** the
  swap (a precondition is a property of the *transition*; `onEnter` runs after, too late to refuse).
- `tryTransition(appState, key, mods)` — global hotkeys `R` / `Ctrl+R` / `P`; returns true if
  handled (performed **or** refused). `keyCallback`: `if (tryTransition(...)) return; else
  currentState->handleKey(...)`.

No `changeMode()` on the interface — transitions are global app commands, not per-mode behavior.
Running them separately from `handleKey` also dodges the self-deletion hazard (a state reassigning
the pointer that owns it mid-method). **Split:** guard (pre-swap) = "may I enter?"; `onEnter`
(post-swap) = "set up now that I'm current." No `onExit` (the destructor covers teardown).

## Clearing matrix (matches oren)

| Entering | `pathPoints` | `waypoints` | camera |
|----------|--------------|-------------|--------|
| RECORD   | clear        | clear       | — |
| PLAYBACK | keep         | keep        | snap to `waypoints[0]` |
| PICK     | clear        | keep        | (seed from a waypoint — with full PICK) |

## Highlight — by camera position, state-independent

`drawWaypoints(waypoints, playerCamera.position)` greens the waypoint whose position matches the
camera, red otherwise — so the highlight tracks the **camera** in any mode and can't desync from
`m_index` (the float compare is exact because PLAYBACK snaps the camera to an exact copy of a
waypoint). `m_index` is purely the navigation cursor.

## Known follow-ups

- **PLAYBACK entry-snap** is a deliberate deviation from oren (snap on enter, not on first key).
- **PICK is a stub.** Real impl adds: `handleMouseButton` (a mouse hook on `State`), the offscreen
  pick pass (Renderer `m_pickShader` seam, fixed-function → core), the ghost overlay
  (`renderPlayerOverlay`), and seeding the camera from a waypoint in `onEnter`. Its guarded +
  parameterized entry is already in place (`requireWaypoints` + `onEnter`).

## Verification

`make` clean; `R` records (`B` adds waypoints, green when the camera is on one); `Ctrl+R` requires
waypoints, snaps to waypoint 0, `UP`/`DOWN` step; `P` requires waypoints, clears the path; arrows /
`<` `>` navigate. With no waypoints, PLAYBACK/PICK are refused with a console message.
