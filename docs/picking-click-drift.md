# Picking: click-then-move records the wrong vertex

Status: **open / not implemented** — recorded for later review.
Context: PICK mode (Mode 2), color-picking correspondences.

## Symptom

In PICK mode, if you move the mouse fast you can left-click, *then* move the
cursor, and the position that gets recorded is where the cursor ended up — not
where you clicked. A quick flick can grab a vertex you only swept across.

## Where the code is

- `PickState::handleMouseButton` — `src/state/States.cpp:151`
  Fires on `LEFT` + `PRESS`, then reads `glfwGetCursorPos` and runs the GPU
  color-pick (`renderer.pickVertex`) at that position.
- Render loop — `src/main.cpp:151` — order is `render → swap → glfwPollEvents()`
  (input is drained once per frame, at the bottom).

## Root cause

It is **one frame of poll latency**, not late sampling:

1. You physically press at time T. The OS queues the event; nothing in our code
   runs yet.
2. The loop only drains input at `glfwPollEvents()`, once per frame.
3. By the time the button callback fires, up to a full frame has passed and the
   cursor has already moved.

Under WSLg the renderer is software (llvmpipe — no GPU passthrough), so the
frame is *fat* (~100–200 ms at 800×600 with the terrain). That fat frame is the
whole window in which "click, then move" registers as motion, which is why it
only shows up "if you're fast enough."

Note: the correspondence stays internally **consistent** — the same (drifted)
cursor feeds both the vertex pick (`worldPos`) and the recorded 2D `imagePos`,
so PnP isn't corrupted. The defect is intent: you aimed at A and recorded B.

## A fix that was considered and rejected: cursor-pos cache

Idea: maintain the cursor position via `glfwSetCursorPosCallback` and read the
cached value in the button handler (emulating SDL/Win32 event-time coordinates).

**Rejected — it's a no-op on GLFW.** GLFW updates `window->virtualCursorPos`
(what `glfwGetCursorPos` returns) on the *same internal line* that invokes the
cursor-pos callback, so the cached value and `glfwGetCursorPos` are byte-for-byte
identical at the same instant. Caching changes nothing.

The genuinely standard fix — coordinates the OS stamps onto the button event
itself (Win32 `lParam`, SDL `event.x/y`) — GLFW does not expose. That door is
closed to us. The only thing that shrinks the latency window is a higher frame
rate (real GPU), which is out of scope.

## Suggested fix: click-vs-drag thresholding

Since we can't make the sample land earlier, reject the samples we know are bad —
exactly how every GUI tells a click from a drag. A click counts only if the
pointer didn't travel between press and release. A fast flick moves between the
two and gets dropped.

- Catches the big errors (the fast-flick case the user reported).
- Bounds the residual to a slow, deliberate click, where one frame of drift is a
  pixel or two.
- Localized to `PickState` — no changes to `main.cpp`, `Callbacks`, or `AppState`.
- Behavior change to note: the pick commits on **release** instead of press
  (which is how real buttons work), and is anchored to the **press** position
  (the point actually aimed at).

### Change 1 — `src/state/States.h`, two fields on `PickState`

```cpp
private:
    void drawPickedPoints(Renderer &renderer, const glm::mat4 &mvp) const;

    bool      m_pressActive = false;   // a left-press is underway
    glm::vec2 m_pressPos{0.0f};        // cursor at press -- to reject drags

    std::vector<PickedPoint> m_pickedPoints;
    std::optional<Waypoint>  m_computedCamera;
```

### Change 2 — `src/state/States.cpp`, replace the body of `handleMouseButton`

```cpp
void PickState::handleMouseButton(AppState &appState, Renderer &renderer,
                                  GLFWwindow *window, int button, int action)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT)
        return;

    double cursorX, cursorY;
    glfwGetCursorPos(window, &cursorX, &cursorY);
    glm::vec2 cursor((float)cursorX, (float)cursorY);

    // Press just remembers where the click started; the pick is committed on RELEASE,
    // and only if the cursor stayed put. A press that travels is a drag/flick, not a
    // deliberate click -- ignoring it stops a fast mouse from recording a vertex it
    // merely swept across. This is the standard GUI click-vs-drag distinction.
    if (action == GLFW_PRESS) {
        m_pressActive = true;
        m_pressPos = cursor;
        return;
    }
    if (action != GLFW_RELEASE || !m_pressActive)
        return;
    m_pressActive = false;

    const float dragThreshold = 4.0f;                  // pixels; a click may jitter this much
    if (glm::distance(m_pressPos, cursor) > dragThreshold)
        return;                                        // travelled -> a drag, not a pick

    // Pick at the PRESS position -- the point the user actually aimed at.
    int id = renderer.pickVertex((int)m_pressPos.x, (int)m_pressPos.y, appState.playerView);
    if (id < 0)
        return;

    const Mesh &mesh = appState.mesh;
    glm::vec3 center(mesh.width / 2.0f, 0.0f, mesh.height / 2.0f);
    glm::vec3 worldPos = mesh.vertices[id].position - center;

    const Viewport &viewport = appState.playerView.viewport;
    glm::vec2 imagePos(m_pressPos.x - viewport.x, m_pressPos.y - viewport.y);

    m_pickedPoints.push_back(PickedPoint{ worldPos, imagePos });
    std::cout << "Picked vertex " << id
              << " world(" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")"
              << " image(" << imagePos.x << ", " << imagePos.y << ")" << std::endl;
}
```

### Knobs / limits

- `dragThreshold = 4.0f` px is the only tunable — raise if real clicks get
  rejected, lower if sloppy ones still slip through.
- Does **not** fix sub-frame drift on a slow deliberate click (bounded to ~1–2
  px; only real GPU acceleration removes it).

## Related

See also the WSLg ghost-window note (interrupted run leaves a window that
survives container rebuild / Docker restart; `wsl --shutdown` clears it without
a reboot) — same WSLg/software-rendering environment is the backdrop for the fat
frame here.
