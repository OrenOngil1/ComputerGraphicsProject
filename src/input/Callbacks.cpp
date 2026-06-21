#include "Callbacks.h"

#include <iostream>
#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>   // glm::rotate (global-map orbit)

#include "../core/Simulation.h"
#include "../state/States.h"
#include "../state/TrackersState.h"
#include "../state/FeatureMatchState.h"

// Is the cursor inside this viewport? The global-map mouse controls only act
// while the cursor is over the global (left) view, so they never fight with the
// player view.
static bool inside(const Viewport &vp, double x, double y)
{
    return x >= vp.x && x < vp.x + vp.width &&
           y >= vp.y && y < vp.y + vp.height;
}

// Scroll-wheel zoom of an overview camera: slide the eye along its view axis,
// keeping the target fixed. Each notch scales the eye-target distance ~10%,
// clamped so it can't cross the target or fly off to infinity.
static void zoomGlobal(Camera &cam, double scrollY)
{
    glm::vec3 forward = cam.target - cam.position;
    float dist = glm::length(forward);
    if (dist < 1e-4f) return;
    forward /= dist;
    dist = glm::clamp(dist * (scrollY > 0 ? 0.9f : 1.0f / 0.9f), 1.0f, 1.0e5f);
    cam.position = cam.target - forward * dist;
}

// Middle-drag pan of an overview camera: shift eye AND target together in the
// camera's view plane, so the map slides under the cursor (grab-the-map feel:
// drag right -> content moves right). Scaled by the eye-target distance so the
// pan feels the same at any zoom.
static void panGlobal(Camera &cam, double dx, double dy)
{
    glm::vec3 forward = glm::normalize(cam.target - cam.position);
    glm::vec3 right   = glm::normalize(glm::cross(forward, cam.up));
    glm::vec3 up      = glm::normalize(glm::cross(right, forward));
    const float scale = glm::length(cam.target - cam.position) * 0.0015f;
    const glm::vec3 delta = right * (float)(-dx) * scale + up * (float)(dy) * scale;
    cam.position += delta;
    cam.target   += delta;
}

// Right-drag orbit of an overview camera: swing the eye around the target so the
// far side of the terrain -- places behind mountains -- can be brought into view.
// Horizontal drag yaws around world up; vertical drag pitches, but only while the
// view stays clear of the vertical so eye/target/up never become colinear. The
// target stays fixed (a pure orbit), so picking still aims at the same scene.
static void orbitGlobal(Camera &cam, double dx, double dy)
{
    glm::vec3 offset = cam.position - cam.target;

    offset = glm::vec3(glm::rotate(glm::mat4(1.0f), (float)(-dx) * 0.005f,
                                   glm::vec3(0.0f, 1.0f, 0.0f)) * glm::vec4(offset, 0.0f));

    glm::vec3 forward = glm::normalize(-offset);
    glm::vec3 right   = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 pitched = glm::vec3(glm::rotate(glm::mat4(1.0f), (float)(-dy) * 0.005f, right)
                                  * glm::vec4(offset, 0.0f));
    if (glm::abs(glm::normalize(pitched).y) < 0.985f)
        offset = pitched;

    cam.position = cam.target + offset;
}

// The one place a transition is performed -- so a state never has to know about
// the states it can transition to -- and the new mode's entry action (onEnter)
// runs in exactly one spot, right after it becomes current.
static void setState(Simulation &sim, std::unique_ptr<State> next)
{
    sim.currentState = std::move(next);
    sim.currentState->onEnter(sim);
}

// PLAYBACK and PICK both require at least one recorded waypoint. The precondition
// is a property of the transition (you may only enter if waypoints exist), so it
// is checked here, before the swap is committed -- not in onEnter, which runs
// after the swap and so would be too late to refuse.
static bool requireWaypoints(const Simulation &sim)
{
    if (sim.waypoints.empty()) {
        std::cout << "Record camera waypoints in RECORD mode first" << std::endl;
        return false;
    }
    return true;
}

// Global, app-level hotkeys that switch mode from anywhere. Returns true if it
// handled the key (transition performed OR refused), so the caller skips in-mode
// handling this event. Running before and separately from the current state's
// handleKey also avoids the self-deletion hazard of a state reassigning the
// pointer that owns it mid-method.
static bool tryTransition(Simulation &sim, int key, int mods)
{
    switch (key) {
        case GLFW_KEY_R:
            if (mods & GLFW_MOD_CONTROL) {
                if (!requireWaypoints(sim))
                    return true;
                setState(sim, std::make_unique<PlaybackState>());
                std::cout << "Switched to PLAYBACK mode" << std::endl;
            } else {
                setState(sim, std::make_unique<RecordState>());
                std::cout << "Switched to RECORD mode" << std::endl;
            }
            return true;

        case GLFW_KEY_P:
            if (!requireWaypoints(sim))
                return true;
            setState(sim, std::make_unique<PickState>());
            std::cout << "Switched to PICK mode" << std::endl;
            return true;

        // No waypoint guard: TRACKERS builds its own ground truth as the user
        // flies and captures, independent of any recording. (Pressing T again
        // re-enters the mode: fresh trackers, fresh log -- like R for RECORD.)
        // The count prompt blocks in the terminal, like the terrain menu; its
        // policy lives with the state, so this case stays a plain transition.
        case GLFW_KEY_T:
            setState(sim, std::make_unique<TrackersState>(TrackersState::promptCount()));
            std::cout << "Switched to TRACKERS mode" << std::endl;
            return true;

        // Waypoint guard like PICK: the recorded waypoints are the pre-phase
        // views the feature database is built from -- without a recording
        // there would be nothing to match against.
        case GLFW_KEY_F:
            if (!requireWaypoints(sim))
                return true;
            setState(sim, std::make_unique<FeatureMatchState>(FeatureMatchState::promptCount()));
            std::cout << "Switched to FEATURE MATCH mode" << std::endl;
            return true;
    }

    return false;
}

// On resize GLFW hands us the new framebuffer size in PIXELS (the units
// glViewport wants -- unlike window size, which is screen coordinates that
// differ from pixels under HiDPI scaling). We recompute the split-screen layout
// here, once per resize, and store each viewport in its View; the render loop
// just reads them. Reaches Simulation through the window user pointer, exactly
// like keyCallback below.
void framebufferSizeCallback(GLFWwindow *window, int width, int height)
{
    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim) {
        std::cerr << "Error: No CallbackContext associated with window" << std::endl;
        return;
    }
    Simulation *sim = ctx->sim;

    sim->globalView.viewport = leftHalf(width, height);
    sim->playerView.viewport = rightHalf(width, height);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    (void)scancode;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim) {
        std::cerr << "Error: No CallbackContext associated with window" << std::endl;
        return;
    }
    Simulation *sim = ctx->sim;

    // Only handle key presses and repeats, ignore releases.
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    // Escape backs out of the current session to the terrain menu (the universal
    // "up one level" key) -- a flag, kept distinct from program exit. Ctrl+Q is the
    // standard "quit the application now": it trips glfwWindowShouldClose, the same
    // signal the OS close button raises, so run() ends the program instead of
    // re-entering the menu.
    if (key == GLFW_KEY_ESCAPE) {
        sim->returnToMenu = true;
        return;
    }
    if (key == GLFW_KEY_Q && (mods & GLFW_MOD_CONTROL)) {
        glfwSetWindowShouldClose(window, true);
        return;
    }

    // L cycles the scene light through the presets. Global like Escape, not a
    // mode transition: the light must be changeable at any moment in any mode
    // (Mode 4's experiment swaps it between its Pre and Run phases).
    if (key == GLFW_KEY_L) {
        std::cout << "Light: " << sim->cycleLightPreset() << std::endl;
        return;
    }

    // A global mode-switch hotkey takes precedence over in-mode handling.
    if (tryTransition(*sim, key, mods))
        return;

    if (sim->currentState)
        sim->currentState->handleKey(*sim, *ctx->renderer, key, mods);
}

// Mouse buttons are routed to the active state unconditionally; the state decides
// whether it cares (only PickState reacts, to a left-click). The Renderer is handed
// through so PICK can run its color-pick render pass in response.
void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    (void)mods;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim) {
        std::cerr << "Error: No CallbackContext associated with window" << std::endl;
        return;
    }

    Simulation *sim = ctx->sim;

    // Middle/right buttons drive the global-map pan/orbit (press over the global
    // view to grab, release to let go), intercepted here so they never reach the
    // active mode.
    if (button == GLFW_MOUSE_BUTTON_MIDDLE || button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            if (inside(sim->globalView.viewport, x, y)) {
                ctx->panning   = (button == GLFW_MOUSE_BUTTON_MIDDLE);
                ctx->rotating  = (button == GLFW_MOUSE_BUTTON_RIGHT);
                ctx->lastDragX = x;
                ctx->lastDragY = y;
            }
        } else if (action == GLFW_RELEASE) {
            ctx->panning = false;
            ctx->rotating = false;
        }
        return;
    }

    if (sim->currentState)
        sim->currentState->handleMouseButton(*sim, *ctx->renderer,
                                                  window, button, action);
}

// Scroll wheel zooms the global (overview) camera while the cursor is over it.
void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    (void)xoffset;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim)
        return;

    double x, y;
    glfwGetCursorPos(window, &x, &y);
    if (inside(ctx->sim->globalView.viewport, x, y))
        zoomGlobal(ctx->sim->globalView.camera, yoffset);
}

// Cursor motion pans (middle-drag) or orbits (right-drag) the global camera.
// Fires constantly, so it stays silent (no error log) and does nothing unless a
// drag is in progress.
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim || (!ctx->panning && !ctx->rotating))
        return;

    const double dx = xpos - ctx->lastDragX;
    const double dy = ypos - ctx->lastDragY;
    if (ctx->panning)
        panGlobal(ctx->sim->globalView.camera, dx, dy);
    else
        orbitGlobal(ctx->sim->globalView.camera, dx, dy);
    ctx->lastDragX = xpos;
    ctx->lastDragY = ypos;
}
