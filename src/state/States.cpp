#include "States.h"

#include <iostream>

#include <GLFW/glfw3.h>

#include "../core/Simulation.h"
#include "../core/Utils.h"        // randomIndex
#include "../input/Movement.h"
#include "../render/Renderer.h"
#include "../vision/Pnp.h"        // computeCameraPose

// Each mode's input + overlay behavior lives here, in its State. The only free
// helper left that is genuinely shared is moveCamera (called by both NavigationState
// and RecordState each frame).
//
// The waypoint overlay highlights whichever waypoint the player camera is
// sitting on (matched by position) -- so the green highlight always tracks the
// camera, in any mode, with no separate cursor to keep in sync.

// Has the camera moved at least `minDist` since the previous sample? Used to thin out
// path recording so a steady glide doesn't append a near-duplicate point every frame.
static bool movedFarEnough(const glm::vec3 &from, const glm::vec3 &to, float minDist)
{
    return glm::distance(from, to) > minDist;
}

// ── NavigationState ──────────────────────────────────────────
void NavigationState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    moveCamera(sim.playerView.camera, sim.terrainSize, window, dt);
}

// ── RecordState ──────────────────────────────────────────────
void RecordState::onEnter(Simulation &sim)
{
    // Start a fresh recording: drop the previous flight path and waypoints.
    sim.pathPoints.clear();
    sim.waypoints.clear();
}

void RecordState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    Camera &playerCamera = sim.playerView.camera;
    const glm::vec3 prevPosition = playerCamera.position;

    moveCamera(playerCamera, sim.terrainSize, window, dt);

    // Capture the new position as a path point once it has drifted far enough from the
    // last one (threshold scales with terrain size so it is resolution-independent).
    float threshold = sim.terrainSize * 0.0001f;
    if (movedFarEnough(prevPosition, playerCamera.position, threshold))
        sim.pathPoints.push_back(playerCamera.position);
}

void RecordState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer; (void)mods;

    // 'B' stores a camera waypoint (position + look-at target).
    if (key == GLFW_KEY_B)
        sim.waypoints.push_back({ sim.playerView.camera.position,
                                       sim.playerView.camera.target });
}

void RecordState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                      const glm::mat4 &mvp) const
{
    renderer.drawPath(sim.pathPoints, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
}

// ── PlaybackState ────────────────────────────────────────────
void PlaybackState::onEnter(Simulation &sim)
{
    // Snap to the first waypoint so PLAYBACK starts on a known pose. The waypoints
    // are guaranteed non-empty here: the transition guard requires them.
    m_index = 0;
    sim.playerView.camera.position = sim.waypoints[m_index].position;
    sim.playerView.camera.target   = sim.waypoints[m_index].target;
}

// UP/DOWN step m_index through the waypoints (wrapping), then the camera snaps to
// the selected waypoint. The overlay highlights by camera position, so the green
// highlight tracks the camera no matter how m_index moves.
void PlaybackState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer; (void)mods;

    const std::vector<Waypoint> &waypoints = sim.waypoints;
    if (waypoints.empty())
        return;

    const size_t n = waypoints.size();
    if (key == GLFW_KEY_UP)
        m_index = (m_index + 1) % n;
    else if (key == GLFW_KEY_DOWN)
        m_index = (m_index + n - 1) % n;   // n - 1 is safe: n >= 1 here
    else
        return;

    sim.playerView.camera.position = waypoints[m_index].position;
    sim.playerView.camera.target   = waypoints[m_index].target;
}

void PlaybackState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    renderer.drawPath(sim.pathPoints, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
}

// ── PickState ────────────────────────────────────────────────

// Signature shade of the PnP estimate: the translucent "ghost" terrain (player
// view) and its position marker (global view) share it, so the dot reads as the
// same thing as the ghost. One constant keeps the two from drifting apart.
static const glm::vec3 estimateColor(1.0f, 0.5f, 0.0f);   // orange (unique to the estimate)
static const float     estimateGhostAlpha = 0.6f;         // ghost blend transparency
static const float     estimateGhostTint  = 0.6f;         // how far the ghost tints toward orange

// Distinct colors so each picked correspondence is identifiable, and the same index
// shows the same color in both views. Cycles if there are more points than entries.
static glm::vec3 pickedPointColor(size_t index)
{
    static const glm::vec3 palette[] = {
        { 1.0f, 1.0f, 1.0f },  // white
        { 1.0f, 1.0f, 0.0f },  // yellow
        { 1.0f, 0.0f, 1.0f },  // magenta
        { 0.0f, 1.0f, 1.0f },  // cyan
        { 0.6f, 0.2f, 0.8f },  // purple
        { 1.0f, 0.4f, 0.7f },  // pink
        { 1.0f, 0.7f, 0.3f },  // peach
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}

void PickState::onEnter(Simulation &sim)
{
    m_pickedPoints.clear();
    m_computedCamera.reset();

    // Seed the player camera at a random recorded waypoint -- the pose the user then
    // tries to recover by picking. PICK is only entered when waypoints exist (the
    // transition guard), so the vector is non-empty.
    const Waypoint &seed = sim.waypoints[randomIndex(sim.waypoints.size())];
    sim.playerView.camera.position = seed.position;
    sim.playerView.camera.target   = seed.target;
}

void PickState::handleMouseButton(Simulation &sim, Renderer &renderer,
                                  GLFWwindow *window, int button, int action)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS)
        return;

    double cursorX, cursorY;
    glfwGetCursorPos(window, &cursorX, &cursorY);

    int id = renderer.pickVertex((int)cursorX, (int)cursorY, sim.playerView);
    if (id < 0)
        return;

    // World position: recenter the picked vertex to match the rendered (centered)
    // world the camera lives in -- mesh.vertices are stored uncentered.
    const Mesh &mesh = sim.mesh;
    glm::vec3 center(mesh.cols / 2.0f, 0.0f, mesh.rows / 2.0f);
    glm::vec3 worldPos = mesh.vertices[id].position - center;

    // Image position: cursor in viewport-local pixels (origin at the viewport corner).
    const Viewport &viewport = sim.playerView.viewport;
    glm::vec2 imagePos((float)cursorX - viewport.x, (float)cursorY - viewport.y);

    m_pickedPoints.push_back(PickedPoint{ worldPos, imagePos });
    std::cout << "Picked vertex " << id
              << " world(" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")"
              << " image(" << imagePos.x << ", " << imagePos.y << ")" << std::endl;
}

void PickState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer; (void)mods;
    if (key != GLFW_KEY_C)
        return;

    const Viewport &viewport = sim.playerView.viewport;
    m_computedCamera = computeCameraPose(m_pickedPoints, sim.playerView.camera.fov,
                                         viewport.width, viewport.height);
}

void PickState::drawPickedPoints(Renderer &renderer, const glm::mat4 &mvp) const
{
    std::vector<glm::vec3> positions, colors;
    positions.reserve(m_pickedPoints.size());
    colors.reserve(m_pickedPoints.size());
    for (size_t i = 0; i < m_pickedPoints.size(); i++) {
        positions.push_back(m_pickedPoints[i].worldPos);
        colors.push_back(pickedPointColor(i));
    }
    renderer.drawPoints(positions, colors, 10.0f, mvp);
}

void PickState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    // Keep the flight context visible while picking -- same as RECORD/PLAYBACK. The
    // seed waypoint the player camera snapped to shows green (the true pose to
    // recover); the red estimate marker below is the PnP guess against it.
    renderer.drawPath(sim.pathPoints, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);

    drawPickedPoints(renderer, mvp);

    // The estimated camera position, once solved -- in the estimate's signature
    // shade so it stands apart from the red waypoint dots.
    if (m_computedCamera)
        renderer.drawPoints({ m_computedCamera->position },
                            { estimateColor }, 5.0f, mvp);
}

void PickState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    if (m_computedCamera) {
        // Terrain as seen from the estimated pose, translucent orange, over the true
        // player view -- the closer the alignment, the better the estimate.
        Camera estimated = sim.playerView.camera;   // inherit fov/near/far/up
        estimated.position = m_computedCamera->position;
        estimated.target   = m_computedCamera->target;
        renderer.drawGhost(estimated, sim.playerView.viewport,
                           estimateColor, estimateGhostAlpha, estimateGhostTint);
    } else {
        drawPickedPoints(renderer, mvp);
    }
}
