#include "States.h"

#include <iostream>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho (PICK's screen-space markers)

#include "OverlayStyle.h"
#include "../core/Simulation.h"
#include "../core/Random.h"       // randomIndex
#include "../input/CameraControls.h"   // fly, MovementIntent
#include "../input/Callbacks.h"        // pollMovementIntent (GLFW glue)
#include "../render/Renderer.h"
#include "../vision/Pnp.h"        // computeCameraPose
#include "../core/FlightPaths.h"   // layOutBuildRing (RECORD's O key)

// Has the camera moved far enough since the previous sample to be worth
// recording? Thins out path recording so a steady glide doesn't append a
// near-duplicate every frame. The threshold is a fraction of terrain size, so
// the recorded path's density is resolution-independent across DEMs.
static bool movedFarEnough(const glm::vec3 &from, const glm::vec3 &to, float terrainSize)
{
    constexpr float kPathSampleFraction = 0.0001f;
    return glm::distance(from, to) > terrainSize * kPathSampleFraction;
}

// ── NavigationState ──────────────────────────────────────────
void NavigationState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    fly(sim.playerView.camera, pollMovementIntent(window), sim.terrainSize, dt);
}

// ── RecordState ──────────────────────────────────────────────
void RecordState::onEnter(Simulation &sim)
{
    sim.pathPoints.clear();
    sim.waypoints.clear();
    m_ringLaid = false;

    std::cout << "RECORD: recording started (previous path and waypoints cleared). "
                 "Fly with WASD / arrows / Q-E; press B to drop a waypoint, or O to"
                 " lay the calibrated 16-station build ring in one stroke." << std::endl;
}

void RecordState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    Camera &playerCamera = sim.playerView.camera;
    const glm::vec3 prevPosition = playerCamera.position;

    fly(playerCamera, pollMovementIntent(window), sim.terrainSize, dt);

    // Flying stays free after O -- looking around is how the ring gets judged
    // -- but the recording is closed: appending here would draw (and, since
    // the path is saved with the database, store) a tail from the ring's
    // closing point to wherever the camera wandered.
    if (!m_ringLaid && movedFarEnough(prevPosition, playerCamera.position, sim.terrainSize))
        sim.pathPoints.push_back(playerCamera.position);
}

void RecordState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer;

    if (key == GLFW_KEY_B) {
        // The ring's spacing IS its calibration, so a stray waypoint is not a
        // small addition to it; say so rather than silently making 17 stations.
        if (m_ringLaid) {
            std::cout << "RECORD: the build ring is laid -- B would add a 17th station"
                         " off its 22.5-degree spacing. Press R to record freehand"
                         " instead." << std::endl;
            return;
        }
        sim.waypoints.push_back(sim.playerView.camera.pose());

        const glm::vec3 &p = sim.playerView.camera.position;
        std::cout << "RECORD: waypoint " << sim.waypoints.size()
                  << " saved at (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }

    // O replaces the freehand flight with the calibrated build ring -- the
    // measured height, spacing, and aim every good manual build shared. The
    // anchoring workflow it feeds is unchanged: F, then G, anchor every other
    // view and Ctrl+X the rest.
    //
    // Bare O only. This discards the recording with no undo, and Ctrl+O is
    // FEATURE MATCH's load-database chord: a hand that arrives here with that
    // habit must not lose a flight to it.
    if (key == GLFW_KEY_O && mods == 0) {
        layOutBuildRing(sim);
        m_ringLaid = true;
        std::cout << "RECORD: build ring laid -- " << sim.waypoints.size()
                  << " stations at the calibrated height and spacing, all aimed at"
                     " the centre (freehand recording is closed; R starts over)."
                     " F starts FEATURE MATCH; in the build, anchor every other"
                     " view and Ctrl+X the rest." << std::endl;
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
    // Waypoints are non-empty here: the transition guard requires them.
    m_index = 0;
    sim.playerView.camera.applyPose(sim.waypoints[m_index]);
}

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
        m_index = (m_index + n - 1) % n;   // wrapping decrement; n >= 1 here
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

// Distinct colors so each picked correspondence is identifiable, with the same
// index showing the same color in both views. Cycles when points outnumber
// entries. White is absent on purpose: it is reserved for the pending marker
// (overlay::pendingPickColor).
static glm::vec3 pickedPointColor(size_t index)
{
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

void PickState::onEnter(Simulation &sim)
{
    m_pickedPoints.clear();
    m_computedCamera.reset();
    m_pendingImageRay.reset();

    // Seed the player camera at a random recorded waypoint -- the pose the user
    // then tries to recover. Waypoints are non-empty (transition guard).
    const Waypoint &seed = sim.waypoints[randomIndex(sim.waypoints.size())];
    sim.playerView.camera.applyPose(seed);

    std::cout << "PICK: camera seeded at a random waypoint -- recover its pose. "
                 "Left-click a 2D point in the player (right) view, then color-pick "
                 "its 3D match in the global (left) view; repeat for at least 4, "
                 "then press C to solve.\n"
                 "      The cyan cone on the map is what the player view sees; the "
                 "white line is where your pending pick's 3D point must lie (judge "
                 "how far along it). X cancels a pending pick, U undoes the last "
                 "correspondence, V cycles the aids (full / cone only / off)."
              << std::endl;
}

// Phase A, the 2D half: must land in the player view AND on terrain. Stored as
// a ray (resize-proof); the color-pick is a validity gate only -- its id is
// discarded, so the 3D point still can't be read off the player view.
void PickState::recordImagePick(Simulation &sim, Renderer &renderer, const glm::dvec2 &cursor)
{
    // Starting a new pair drops any previous solve, so the overlays go back to
    // showing the correspondences being built rather than a ghost computed
    // from a set the user is still editing.
    m_computedCamera.reset();

    const Viewport &viewport = sim.playerView.viewport;
    if (!viewport.contains(cursor.x, cursor.y)) {
        std::cout << "PICK: click a 2D point in the player (right) view first." << std::endl;
        return;
    }
    if (renderer.pickVertex((int)cursor.x, (int)cursor.y, sim.playerView) < 0) {
        std::cout << "PICK: no terrain under the cursor -- click a 2D point on the terrain." << std::endl;
        return;
    }

    glm::vec2 uv(((float)cursor.x - viewport.x) / (float)viewport.width,
                 ((float)cursor.y - viewport.y) / (float)viewport.height);
    m_pendingImageRay = fractionToRay(uv, sim.playerView.camera.fov, viewport.aspect());
    std::cout << "PICK: 2D recorded at ray(" << m_pendingImageRay->x << ", " << m_pendingImageRay->y
              << ") -- now color-pick its 3D point in the global (left) view." << std::endl;
}

// Phase B, the 3D half: color-pick the vertex under the cursor in the global
// view (reading the map), completing the pending pair.
void PickState::recordWorldPick(Simulation &sim, Renderer &renderer, const glm::dvec2 &cursor)
{
    const Viewport &viewport = sim.globalView.viewport;
    if (!viewport.contains(cursor.x, cursor.y)) {
        std::cout << "PICK: color-pick the matching 3D point in the global (left) view." << std::endl;
        return;
    }

    int id = renderer.pickVertex((int)cursor.x, (int)cursor.y, sim.globalView);
    if (id < 0) {
        // Miss (sky / off-terrain): keep the pending 2D so the user can retry.
        std::cout << "PICK: no terrain under the cursor -- try the 3D pick again." << std::endl;
        return;
    }

    glm::vec3 worldPos = sim.mesh.worldPos(id);

    m_pickedPoints.push_back(Observation{ worldPos, *m_pendingImageRay });
    std::cout << "PICK: correspondence " << m_pickedPoints.size() << ": vertex " << id
              << " world(" << worldPos.x << ", " << worldPos.y << ", " << worldPos.z << ")"
              << " ray(" << m_pendingImageRay->x << ", " << m_pendingImageRay->y << ")"
              << std::endl;
    m_pendingImageRay.reset();
}

void PickState::handleMouseButton(Simulation &sim, Renderer &renderer,
                                  GLFWwindow *window, int button, int action)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS)
        return;

    const glm::dvec2 cursor = cursorPosPixels(window);
    if (!m_pendingImageRay)
        recordImagePick(sim, renderer, cursor);
    else
        recordWorldPick(sim, renderer, cursor);
}

void PickState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)renderer; (void)mods;

    switch (key) {
        // X abandons a half-finished pair -- the same "drop the thing you are
        // being asked about" key FEATURE MATCH's build uses to skip.
        case GLFW_KEY_X:
            if (!m_pendingImageRay) {
                std::cout << "PICK: no pending 2D pick to cancel." << std::endl;
                return;
            }
            m_pendingImageRay.reset();
            std::cout << "PICK: pending 2D pick cancelled." << std::endl;
            return;

        // U removes the last completed correspondence. Without it a misclick
        // is permanent, and one bad pair is enough to wreck the solve.
        case GLFW_KEY_U:
            if (m_pickedPoints.empty()) {
                std::cout << "PICK: nothing to undo." << std::endl;
                return;
            }
            m_pickedPoints.pop_back();
            m_computedCamera.reset();   // that solve no longer describes this set
            std::cout << "PICK: undid the last correspondence -- "
                      << m_pickedPoints.size() << " left." << std::endl;
            return;

        case GLFW_KEY_C:
            solveFromPicks(sim);
            return;

        default:
            return;
    }
}

void PickState::solveFromPicks(const Simulation &sim)
{
    // Project each stored ray into the CURRENT player viewport: ray -> fraction
    // -> pixels + intrinsics (all at this aspect, inside computeCameraPose).
    // Consistent even if the window was resized between picking and solving.
    const Viewport &viewport = sim.playerView.viewport;
    const float fov = sim.playerView.camera.fov;
    const float aspect = viewport.aspect();

    std::vector<Correspondence> correspondences;
    correspondences.reserve(m_pickedPoints.size());
    for (const Observation &o : m_pickedPoints)
        correspondences.push_back(Correspondence{ o.worldPos, rayToFraction(o.imageRay, fov, aspect) });

    m_computedCamera = computeCameraPose(correspondences, fov, viewport.width, viewport.height);

    // The player camera never moves in PICK (no tick override), so its current
    // pose is still the seeded ground truth to report the error against.
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

// The 2D observations, drawn where the user clicked: each stored ray is
// reprojected into the CURRENT aspect (so markers survive a resize), never as
// the reprojection of worldPos (which would move the marker and leak the true
// projection). An ortho over the unit square (top-left origin, y down) maps
// the [0,1] fractions straight to the viewport.
void PickState::drawImageMarkers(Renderer &renderer, float fov, const Viewport &viewport) const
{
    const float aspect = viewport.aspect();
    const glm::mat4 screen = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);

    std::vector<glm::vec3> positions, colors;
    positions.reserve(m_pickedPoints.size());
    colors.reserve(m_pickedPoints.size());
    for (size_t i = 0; i < m_pickedPoints.size(); i++) {
        positions.push_back(glm::vec3(rayToFraction(m_pickedPoints[i].imageRay, fov, aspect), 0.0f));
        colors.push_back(pickedPointColor(i));
    }
    if (!positions.empty())
        renderer.drawPoints(positions, colors, overlay::pickMarkerSize, screen);

    // The pending pick rides along at the same size; its reserved white color
    // alone marks it as the one awaiting its 3D match.
    if (m_pendingImageRay)
        renderer.drawPoints({ glm::vec3(rayToFraction(*m_pendingImageRay, fov, aspect), 0.0f) },
                            { overlay::pendingPickColor }, overlay::pickMarkerSize, screen);
}

// Map-side aids for the pairs being built: the pending observation's line is
// the same white as its player-view marker so the two read as one object;
// completed ones are dim and double as a self-check (a 3D marker off its own
// line was mis-picked -- U removes it). Lines give DIRECTION only, never an
// intersection with the terrain: where along one the point sits is exactly
// the manual step.
void PickState::drawSightAids(const Simulation &sim, Renderer &renderer,
                              const glm::mat4 &mvp) const
{
    if (sim.viewAids != ViewAids::Full)
        return;

    const Camera &camera = sim.playerView.camera;
    const float reach = sim.terrainSize * 1.5f;   // clears the map from anywhere on it

    if (!m_pickedPoints.empty()) {
        std::vector<glm::vec2> rays;
        rays.reserve(m_pickedPoints.size());
        for (const Observation &observation : m_pickedPoints)
            rays.push_back(observation.imageRay);

        renderer.drawSightLines(camera, rays, reach, overlay::sightLineDimColor,
                                overlay::sightLineDimWidth, mvp);
    }

    if (m_pendingImageRay)
        renderer.drawSightLines(camera, { *m_pendingImageRay }, reach,
                                overlay::pendingPickColor, overlay::sightLineWidth, mvp);
}

void PickState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    // Keep the flight context visible while picking; the seed waypoint the
    // camera sits on shows green (the true pose to recover).
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);

    if (m_computedCamera) {
        renderer.drawPoints({ m_computedCamera->position },
                            { overlay::estimateColor }, overlay::estimateMarkerSize, mvp);
        return;
    }

    // Aids first, markers second: both draw with depth off, so paint order is
    // what keeps the picked points readable on top of their lines.
    drawSightAids(sim, renderer, mvp);
    drawWorldMarkers(renderer, mvp);
}

void PickState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                    const glm::mat4 &mvp) const
{
    // Both player overlays are screen-space with their own matrices; the scene
    // mvp goes unused here.
    (void)mvp;

    if (m_computedCamera) {
        // Terrain as seen from the estimated pose, translucent, over the true
        // view -- the closer the alignment, the better the estimate.
        Camera estimated = sim.playerView.camera;   // inherit fov/near/far/up
        estimated.applyPose(*m_computedCamera);
        renderer.drawGhost(estimated, sim.playerView.viewport,
                           overlay::estimateColor, overlay::estimateGhostAlpha,
                           overlay::estimateGhostTint);
    } else {
        drawImageMarkers(renderer, sim.playerView.camera.fov, sim.playerView.viewport);
    }
}
