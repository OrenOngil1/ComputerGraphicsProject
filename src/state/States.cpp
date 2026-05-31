#include "States.h"

#include <GLFW/glfw3.h>

#include "../core/AppState.h"
#include "../input/Movement.h"
#include "../render/Renderer.h"

// Each mode's input + overlay behavior lives here, in its State. The only free
// helpers left are genuinely shared: handleMovement (used by both NavigationState
// and RecordState) and the Overlay drawing functions.
//
// The waypoint overlay highlights whichever waypoint the player camera is
// sitting on (matched by position) -- so the green highlight always tracks the
// camera, in any mode, with no separate cursor to keep in sync.

// ── NavigationState ──────────────────────────────────────────
void NavigationState::handleKey(AppState &appState, int key, int mods)
{
    handleMovement(appState.playerCamera, appState.terrainSize, key, mods);
}

// ── RecordState ──────────────────────────────────────────────
void RecordState::onEnter(AppState &appState)
{
    // Start a fresh recording: drop the previous flight path and waypoints.
    appState.pathPoints.clear();
    appState.waypoints.clear();
}

void RecordState::handleKey(AppState &appState, int key, int mods)
{
    const glm::vec3 prevPosition = appState.playerCamera.position;
    handleMovement(appState.playerCamera, appState.terrainSize, key, mods);

    // Capture the new position as a path point whenever the camera actually moved.
    if (appState.playerCamera.position != prevPosition)
        appState.pathPoints.push_back(appState.playerCamera.position);

    // 'B' stores a camera waypoint (position + look-at target).
    if (key == GLFW_KEY_B)
        appState.waypoints.push_back({ appState.playerCamera.position,
                                           appState.playerCamera.target });
}

void RecordState::renderGlobalOverlay(const AppState &appState, Renderer &renderer,
                                      const glm::mat4 &mvp) const
{
    renderer.drawPath(appState.pathPoints, mvp);
    renderer.drawWaypoints(appState.waypoints, appState.playerCamera.position, mvp);
}

// ── PlaybackState ────────────────────────────────────────────
void PlaybackState::onEnter(AppState &appState)
{
    // Snap to the first waypoint so PLAYBACK starts on a known pose. The waypoints
    // are guaranteed non-empty here: the transition guard requires them.
    m_index = 0;
    appState.playerCamera.position = appState.waypoints[m_index].position;
    appState.playerCamera.target   = appState.waypoints[m_index].target;
}

// UP/DOWN step m_index through the waypoints (wrapping), then the camera snaps to
// the selected waypoint. The overlay highlights by camera position, so the green
// highlight tracks the camera no matter how m_index moves.
void PlaybackState::handleKey(AppState &appState, int key, int mods)
{
    (void)mods;
    const std::vector<Waypoint> &waypoints = appState.waypoints;
    if (waypoints.empty())
        return;

    const size_t n = waypoints.size();
    if (key == GLFW_KEY_UP)
        m_index = (m_index + 1) % n;
    else if (key == GLFW_KEY_DOWN)
        m_index = (m_index + n - 1) % n;   // n - 1 is safe: n >= 1 here
    else
        return;

    appState.playerCamera.position = waypoints[m_index].position;
    appState.playerCamera.target   = waypoints[m_index].target;
}

void PlaybackState::renderGlobalOverlay(const AppState &appState, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    renderer.drawPath(appState.pathPoints, mvp);
    renderer.drawWaypoints(appState.waypoints, appState.playerCamera.position, mvp);
}

// ── PickState ────────────────────────────────────────────────
void PickState::onEnter(AppState &appState)
{
    // Clear the flight path for a clean picking view (matches oren). The camera
    // seed from a recorded pose comes with the full PICK implementation.
    appState.pathPoints.clear();
}
