# WSLg ghost window: a stuck run leaves a window that won't close

Status: **environment note / known workaround** — recorded for later review.
Context: running `bin/drone_sim` (GLFW window) under WSL2 + Docker Desktop.

## Symptom

A run got stuck and was interrupted, but the GLFW window stayed on screen. It
survived **everything** short of a full reboot:

- Rebuilding the container (no cache) — no effect.
- Closing/reopening the folder in VS Code — no effect.
- Restarting Docker Desktop — no effect.
- Restarting the computer — window finally gone.

## Why it happens

The window is **not owned by the container, and not by Docker.** On WSL2 +
Docker Desktop the GUI surface is drawn by **WSLg** — a compositor (Weston) plus
an X server running in their own dedicated WSL *system distro* on the Windows
side. Our process is just a client that opens a socket to that compositor and
asks it to show a surface.

That explains every observation:

| Attempt                    | Why it didn't help                                              |
| -------------------------- | --------------------------------------------------------------- |
| Rebuild container (no cache) | Rebuilds the image; never touches the running compositor.     |
| Restart Docker Desktop     | Restarts Docker's own WSL2 backend distros; WSLg is separate.   |
| Restart computer           | Tears down everything including WSLg → surface finally gone.     |

**Why it got orphaned:** the stuck process almost certainly did not exit cleanly
through `glfwTerminate()` (`src/main.cpp:163`). Either it was wedged in a
blocking call (a stalled `glfwSwapBuffers` on the software renderer, or sitting
in `std::cin >> choice` in `selectTerrain` before the loop ever started — which
*looks* like a hang if you expect a GUI), or it was hard-killed. Either way the
graceful "destroy my surface" handshake never ran, so WSLg kept the ghost.

## The fix you actually want (no reboot)

From a **Windows** PowerShell/cmd (not inside the container):

```
wsl --shutdown
```

This stops all WSL2 distros including the WSLg system distro, which resets the
compositor and drops the ghost window in a couple of seconds. Reach for this
before ever rebooting again.

## Reducing the chance it happens

Mostly a WSLg robustness quirk, not a bug in our code — but levers on our side:

- **`std::cin >> choice` before the window opens** (`selectTerrain`,
  `src/core/Menu.cpp:40`) is the most likely "it's stuck!" culprit in this
  codebase: it blocks on console input before the GLFW window exists, which looks
  like a frozen GUI. Worth knowing it's there.
- **Optional polish:** a small `SIGINT`/`SIGTERM` handler that calls
  `glfwSetWindowShouldClose(window, true)` lets Ctrl-C end the loop *gracefully*
  through the existing teardown (`main.cpp:141`–`163`) instead of leaving a
  half-killed client holding a surface. This is the only real prevention lever on
  our side; treat it as optional, not urgent.

## Related

Same WSLg + software-rendering (llvmpipe) environment is the backdrop for the
picking click-drift issue — see `picking-click-drift.md` (the fat software frame
that causes the click latency is the same renderer that can stall `glfwSwapBuffers`
here).
