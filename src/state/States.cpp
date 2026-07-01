#include "States.h"

#include <cmath>
#include <iostream>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho (PICK's screen-space 2D marker)

#include "OverlayStyle.h"
#include "../core/Simulation.h"
#include "../core/Utils.h"        // randomIndex
#include "../input/CameraControls.h"   // fly, MovementIntent
#include "../input/Callbacks.h"        // pollMovementIntent (GLFW glue)
#include "../render/Renderer.h"
#include "../vision/Pnp.h"        // computeCameraPose

// Each mode's input + overlay behavior lives here, in its State. The only free
// helper left that is genuinely shared is fly (called by both NavigationState
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
    fly(sim.playerView.camera, pollMovementIntent(window), sim.terrainSize, dt);
}

// ── RecordState ──────────────────────────────────────────────
void RecordState::onEnter(Simulation &sim)
{
    // Start a fresh recording: drop the previous flight path and waypoints.
    sim.pathPoints.clear();
    sim.waypoints.clear();

    std::cout << "RECORD: recording started (previous path and waypoints cleared). "
                 "Fly with WASD / arrows / Q-E; press B to drop a waypoint." << std::endl;
}

void RecordState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    Camera &playerCamera = sim.playerView.camera;
    const glm::vec3 prevPosition = playerCamera.position;

    fly(playerCamera, pollMovementIntent(window), sim.terrainSize, dt);

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
    if (key == GLFW_KEY_B) {
        sim.waypoints.push_back({ sim.playerView.camera.position,
                                       sim.playerView.camera.target });

        const glm::vec3 &p = sim.playerView.camera.position;
        std::cout << "RECORD: waypoint " << sim.waypoints.size()
                  << " saved at (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }
}

void RecordState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                      const glm::mat4 &mvp) const
{
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
}

// ── PlaybackState ────────────────────────────────────────────
void PlaybackState::onEnter(Simulation &sim)
{
    // Snap to the first waypoint so PLAYBACK starts on a known pose. The waypoints
    // are guaranteed non-empty here: the transition guard requires them.
    m_index = 0;
    sim.playerView.camera.applyPose(sim.waypoints[m_index]);
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

    sim.playerView.camera.applyPose(waypoints[m_index]);
}

void PlaybackState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
}

// ── PickState ────────────────────────────────────────────────

// Distinct colors so each picked correspondence is identifiable, and the same index
// shows the same color in both views. Cycles if there are more points than entries.
static glm::vec3 pickedPointColor(size_t index)
{
    // White is deliberately absent: it is reserved for the pending marker
    // (overlay::pendingPickColor), so a completed correspondence is never confusable
    // with the in-progress one.
    static const glm::vec3 palette[] = {
        { 1.0f, 1.0f, 0.0f },  // yellow
        { 1.0f, 0.0f, 1.0f },  // magenta
        { 0.0f, 1.0f, 1.0f },  // cyan
        { 0.6f, 0.2f, 0.8f },  // purple
        { 1.0f, 0.4f, 0.7f },  // pink
        { 1.0f, 0.7f, 0.3f },  // peach
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}

// PICK stores observations as camera-space viewing rays so they survive a window
// resize (see PickState::Observation). These convert between a viewport fraction
// ([0,1], origin top-left) and that ray, for a vertical-FOV perspective camera at a
// given aspect -- the same mapping glm::perspective and getCameraIntrinsicMatrix use:
//   ray = ((u - 0.5)*2*tan(fov/2)*aspect, (v - 0.5)*2*tan(fov/2)).
// Only the horizontal term carries the aspect (glm fixes the vertical FOV) -- which is
// exactly why a stored fraction goes stale horizontally when the window is resized.
static glm::vec2 fractionToRay(glm::vec2 uv, float fovDeg, float aspect)
{
    const float t = std::tan(glm::radians(fovDeg) * 0.5f);
    return { (uv.x - 0.5f) * 2.0f * t * aspect, (uv.y - 0.5f) * 2.0f * t };
}

static glm::vec2 rayToFraction(glm::vec2 ray, float fovDeg, float aspect)
{
    const float t = std::tan(glm::radians(fovDeg) * 0.5f);
    return { 0.5f + ray.x / (2.0f * t * aspect), 0.5f + ray.y / (2.0f * t) };
}

void PickState::onEnter(Simulation &sim)
{
    m_pickedPoints.clear();
    m_computedCamera.reset();
    m_pendingImageRay.reset();

    // Seed the player camera at a random recorded waypoint -- the pose the user then
    // tries to recover by picking. PICK is only entered when waypoints exist (the
    // transition guard), so the vector is non-empty.
    const Waypoint &seed = sim.waypoints[randomIndex(sim.waypoints.size())];
    sim.playerView.camera.applyPose(seed);

    std::cout << "PICK: camera seeded at a random waypoint -- recover its pose. "
                 "Left-click a 2D point in the player (right) view, then color-pick "
                 "its 3D match in the global (left) view; repeat for at least 4, "
                 "then press C to solve." << std::endl;
}

void PickState::handleMouseButton(Simulation &sim, Renderer &renderer,
                                  GLFWwindow *window, int button, int action)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS)
        return;

    double cursorX, cursorY;
    glfwGetCursorPos(window, &cursorX, &cursorY);

    // A correspondence is built in two clicks: first the 2D observation in the player
    // (camera) view, then the matching 3D point color-picked in the global view (the
    // "map"). m_pendingImagePos tells the two phases apart -- empty means we still
    // need the 2D half; set means we are awaiting its 3D match.
    if (!m_pendingImageRay) {
        // Phase A: the 2D half. Must land in the player view. Convert the cursor to a
        // viewport fraction, then to the camera-space viewing ray (see Observation) so
        // the observation survives a later resize -- crucially, we do NOT color-pick
        // here, so the 3D point can't be read off the player view.
        const Viewport &viewport = sim.playerView.viewport;
        if (!viewport.contains(cursorX, cursorY)) {
            std::cout << "PICK: click a 2D point in the player (right) view first." << std::endl;
            return;
        }

        glm::vec2 uv(((float)cursorX - viewport.x) / (float)viewport.width,
                     ((float)cursorY - viewport.y) / (float)viewport.height);
        m_pendingImageRay = fractionToRay(uv, sim.playerView.camera.fov, viewport.aspect());
        std::cout << "PICK: 2D recorded at ray(" << m_pendingImageRay->x << ", " << m_pendingImageRay->y
                  << ") -- now color-pick its 3D point in the global (left) view." << std::endl;
        return;
    }

    // Phase B: the 3D half. Must land in the global view, where we color-pick the
    // vertex under the cursor (reading the map).
    const Viewport &viewport = sim.globalView.viewport;
    if (!viewport.contains(cursorX, cursorY)) {
        std::cout << "PICK: color-pick the matching 3D point in the global (left) view." << std::endl;
        return;
    }

    int id = renderer.pickVertex((int)cursorX, (int)cursorY, sim.globalView);
    if (id < 0) {
        // A miss (sky / off-terrain): keep the pending 2D so the user can retry the 3D.
        std::cout << "PICK: no terrain under the cursor -- try the 3D pick again." << std::endl;
        return;
    }

    // World position: recenter the picked vertex to match the rendered (centered)
    // world the camera lives in -- mesh.vertices are stored uncentered.
    glm::vec3 worldPos = sim.mesh.worldPos(id);

    m_pickedPoints.push_back(Observation{ worldPos, *m_pendingImageRay });
    std::cout << "PICK: correspondence " << m_pickedPoints.size() << ": vertex " << id
              << " world(" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")"
              << " ray(" << m_pendingImageRay->x << ", " << m_pendingImageRay->y << ")"
              << std::endl;
    m_pendingImageRay.reset();
}

void PickState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer; (void)mods;
    if (key != GLFW_KEY_C)
        return;

    // Project each stored ray back into the CURRENT player viewport: ray ->
    // fraction (this aspect) -> pixels + intrinsics (also this aspect, inside
    // computeCameraPose). Because the ray is aspect-invariant, this stays consistent
    // even if the window was resized between picking and solving.
    const Viewport &viewport = sim.playerView.viewport;
    const float fov = sim.playerView.camera.fov;
    const float aspect = viewport.aspect();

    std::vector<Correspondence> correspondences;
    correspondences.reserve(m_pickedPoints.size());
    for (const Observation &o : m_pickedPoints)
        correspondences.push_back(Correspondence{ o.worldPos, rayToFraction(o.imageRay, fov, aspect) });

    m_computedCamera = computeCameraPose(correspondences, fov, viewport.width, viewport.height);

    // The player camera never moves in PICK (no tick override), so its current pose
    // is still the seeded ground truth -- report the estimate's error against it, the
    // same position-error readout the other pose modes print on capture.
    if (m_computedCamera) {
        const glm::vec3 err = m_computedCamera->position - sim.playerView.camera.position;
        std::cout << "PICK: pose computed from " << m_pickedPoints.size()
                  << " correspondences, position error " << glm::length(err) << std::endl;
    } else if (m_pickedPoints.size() < 4) {
        std::cout << "PICK: pose NOT computed -- need at least 4 correspondences (have "
                  << m_pickedPoints.size() << ")" << std::endl;
    } else {
        std::cout << "PICK: pose NOT computed -- the solver failed on these "
                  << m_pickedPoints.size() << " correspondences" << std::endl;
    }
}

// Global view (the "map"): the 3D half of each correspondence, in world space.
void PickState::drawWorldMarkers(Renderer &renderer, const glm::mat4 &mvp) const
{
    std::vector<glm::vec3> positions, colors;
    positions.reserve(m_pickedPoints.size());
    colors.reserve(m_pickedPoints.size());
    for (size_t i = 0; i < m_pickedPoints.size(); i++) {
        positions.push_back(m_pickedPoints[i].worldPos);
        colors.push_back(pickedPointColor(i));
    }
    renderer.drawPoints(positions, colors, overlay::pickMarkerSize, mvp);
}

// Player view (the camera): the 2D half of each correspondence -- the user's
// observation -- drawn where they clicked, in normalized screen space. Crucially we
// draw imagePos, NOT the reprojection of worldPos: the observation must stay put, and
// reprojecting the 3D point would both move the marker and leak its true projection.
// An orthographic matrix over the unit square (top-left origin, y down) maps the
// stored [0,1] coordinates straight to the viewport, so markers also track resizes.
// The pending pick (2D clicked, 3D not yet) rides along here in the pending color.
void PickState::drawImageMarkers(Renderer &renderer, float fov, const Viewport &viewport) const
{
    const float aspect = viewport.aspect();
    const glm::mat4 screen = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);

    // Completed observations: palette-colored, standard size (matching the map markers).
    // Each stored ray is reprojected into the CURRENT aspect, so markers stay on their
    // feature through a resize instead of drifting (a frozen fraction would slide).
    std::vector<glm::vec3> positions, colors;
    positions.reserve(m_pickedPoints.size());
    colors.reserve(m_pickedPoints.size());
    for (size_t i = 0; i < m_pickedPoints.size(); i++) {
        positions.push_back(glm::vec3(rayToFraction(m_pickedPoints[i].imageRay, fov, aspect), 0.0f));
        colors.push_back(pickedPointColor(i));
    }
    if (!positions.empty())
        renderer.drawPoints(positions, colors, overlay::pickMarkerSize, screen);

    // The pending pick (2D clicked, 3D not yet) is drawn in its own pass, same size as
    // the completed markers: the reserved white color (pendingPickColor) alone marks it
    // as the not-yet-confirmed correspondence awaiting its 3D match.
    if (m_pendingImageRay)
        renderer.drawPoints({ glm::vec3(rayToFraction(*m_pendingImageRay, fov, aspect), 0.0f) },
                            { overlay::pendingPickColor }, overlay::pickMarkerSize, screen);
}

void PickState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    // Keep the flight context visible while picking -- same as RECORD/PLAYBACK. The
    // seed waypoint the player camera snapped to shows green (the true pose to
    // recover); the red estimate marker below is the PnP guess against it.
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);

    // The estimated camera position, once solved -- in the estimate's signature
    // shade so it stands apart from the red waypoint dots.
    if (m_computedCamera)
        renderer.drawPoints({ m_computedCamera->position },
                            { overlay::estimateColor }, overlay::estimateMarkerSize, mvp);
    else
        drawWorldMarkers(renderer, mvp);
}

void PickState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    // mvp is the player scene matrix; the player overlay is either the screen-space
    // ghost (its own matrix) or screen-space 2D markers (drawImageMarkers' own ortho),
    // so the scene mvp goes unused here -- unlike the global overlay, which needs it.
    (void)mvp;

    if (m_computedCamera) {
        // Terrain as seen from the estimated pose, translucent orange, over the true
        // player view -- the closer the alignment, the better the estimate.
        Camera estimated = sim.playerView.camera;   // inherit fov/near/far/up
        estimated.applyPose(*m_computedCamera);
        renderer.drawGhost(estimated, sim.playerView.viewport,
                           overlay::estimateColor, overlay::estimateGhostAlpha,
                           overlay::estimateGhostTint);
    } else {
        // The 2D observations (completed + pending), each where the user clicked.
        drawImageMarkers(renderer, sim.playerView.camera.fov, sim.playerView.viewport);
    }
}
