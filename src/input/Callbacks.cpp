#include "Callbacks.h"

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>

#include "../core/Simulation.h"
#include "../state/States.h"
#include "../state/TrackersState.h"
#include "../state/FeatureMatchState.h"
#include "../vision/FeatureDbIo.h"   // featureDbPath: F may enter to load a saved db

// 1 while `key` is physically held, else 0 -- int, so opposing keys cancel via
// plain subtraction below.
static int held(GLFWwindow *window, int key)
{
    return glfwGetKey(window, key) == GLFW_PRESS ? 1 : 0;
}

// Continuous movement is a polling model: every frame, ask GLFW which keys are
// down right now and integrate over dt (in fly()). Thin GLFW glue -- the
// motion math lives in CameraControls and is testable without a window.
MovementIntent pollMovementIntent(GLFWwindow *window)
{
    MovementIntent in;
    in.forward  = held(window, GLFW_KEY_W)     - held(window, GLFW_KEY_S);
    in.strafe   = held(window, GLFW_KEY_D)     - held(window, GLFW_KEY_A);
    in.vertical = held(window, GLFW_KEY_Q)     - held(window, GLFW_KEY_E);
    in.yaw      = held(window, GLFW_KEY_RIGHT) - held(window, GLFW_KEY_LEFT);
    in.pitch    = held(window, GLFW_KEY_UP)    - held(window, GLFW_KEY_DOWN);
    return in;
}

// Screen coords -> framebuffer pixels via the window-to-framebuffer ratio
// (1:1 on Windows/X11, 2:1 on e.g. Retina). Zero window size (minimized)
// leaves the coords unscaled -- no viewport contains them anyway.
glm::dvec2 cursorPosPixels(GLFWwindow *window)
{
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    int winWidth, winHeight, fbWidth, fbHeight;
    glfwGetWindowSize(window, &winWidth, &winHeight);
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    if (winWidth > 0 && winHeight > 0) {
        x *= (double)fbWidth / winWidth;
        y *= (double)fbHeight / winHeight;
    }
    return { x, y };
}

// The one place a transition is performed, so the new mode's entry action runs
// in exactly one spot, right after it becomes current.
static void setState(Simulation &sim, std::unique_ptr<State> next)
{
    sim.currentState = std::move(next);
    sim.currentState->onEnter(sim);
}

// Shared transition guard: PLAYBACK and PICK need at least one recorded
// waypoint (FEATURE MATCH too, but with its own exception below). Checked
// before the swap is committed -- onEnter runs after it and would be too late
// to refuse.
static bool requireWaypoints(const Simulation &sim)
{
    if (sim.waypoints.empty()) {
        std::cout << "Record camera waypoints in RECORD mode first" << std::endl;
        return false;
    }
    return true;
}

// App-level hotkeys that switch mode from anywhere. Returns true if it handled
// the key (transition performed OR refused), so the caller skips in-mode
// handling. Running before the current state's handleKey also avoids the
// self-deletion hazard of a state reassigning the pointer that owns it.
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
        // flies and captures. Pressing T again re-enters: fresh trackers,
        // fresh log.
        case GLFW_KEY_T:
            setState(sim, std::make_unique<TrackersState>(TrackersState::promptCount()));
            std::cout << "Switched to TRACKERS mode" << std::endl;
            return true;

        // Waypoint guard like PICK -- except when a saved database exists for
        // this terrain. Its waypoints were saved with it and come back on
        // Ctrl+O, so demanding a throwaway recording first would only replace
        // the views the database was actually built from. Without this
        // exception, a fresh run could never reach Ctrl+O at all.
        case GLFW_KEY_F: {
            if (sim.waypoints.empty() &&
                !std::filesystem::exists(featureDbPath(sim.terrainFile))) {
                std::cout << "Record camera waypoints in RECORD mode first"
                          << std::endl;
                return true;
            }
            // Sequenced rather than nested in the make_unique call: as two
            // arguments, the order they PRINT in would be unspecified.
            const size_t features = FeatureMatchState::promptCount();
            const std::optional<size_t> inliers = FeatureMatchState::promptMinInliers();
            setState(sim, std::make_unique<FeatureMatchState>(features, inliers));
            std::cout << "Switched to FEATURE MATCH mode" << std::endl;
            if (sim.waypoints.empty())
                std::cout << "FEATURES: no recorded views -- Ctrl+O loads the saved"
                             " database, and the views it was built from come back"
                             " with it" << std::endl;
            return true;
        }
    }

    return false;
}

// GLFW reports the new framebuffer size in PIXELS (what glViewport wants --
// unlike window size, which is screen coordinates that differ under HiDPI).
// Recompute the split-screen layout once per resize; the render loop reads it.
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

    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    Simulation *sim = ctx->sim;

    // Escape backs out of the session to the terrain menu; Ctrl+Q quits the
    // program via glfwWindowShouldClose, the same signal as the OS close button.
    if (key == GLFW_KEY_ESCAPE) {
        sim->returnToMenu = true;
        return;
    }
    if (key == GLFW_KEY_Q && (mods & GLFW_MOD_CONTROL)) {
        glfwSetWindowShouldClose(window, true);
        return;
    }

    // L cycles the light presets. Global like Escape, not a mode transition:
    // the light must be changeable in any mode (Mode 4's experiment swaps it
    // between its pre and run phases).
    if (key == GLFW_KEY_L) {
        std::cout << "Light: " << sim->cycleLightPreset() << std::endl;
        return;
    }

    // V cycles the global map's view aids: full -> cone only -> off. Global
    // like L rather than a mode key: the cone belongs to the map, and every
    // mode draws the same one.
    if (key == GLFW_KEY_V) {
        std::cout << "View aids: " << sim->cycleViewAids() << std::endl;
        return;
    }

    // Zero-sized framebuffer (minimized): the viewports are degenerate, and
    // transitions and in-mode keys both do viewport math -- drop the key until
    // the next resize. The global keys above stay usable.
    if (sim->playerView.viewport.width <= 0 || sim->playerView.viewport.height <= 0)
        return;

    // Mode-switch hotkeys take precedence over in-mode handling.
    if (tryTransition(*sim, key, mods))
        return;

    if (sim->currentState)
        sim->currentState->handleKey(*sim, *ctx->renderer, key, mods);
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    (void)mods;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim) {
        std::cerr << "Error: No CallbackContext associated with window" << std::endl;
        return;
    }

    Simulation *sim = ctx->sim;

    // Middle/right buttons drive the global-map pan/orbit (press over the
    // global view to grab, release to let go), intercepted here so they never
    // reach the active mode.
    if (button == GLFW_MOUSE_BUTTON_MIDDLE || button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            const glm::dvec2 cursor = cursorPosPixels(window);
            if (sim->globalView.viewport.contains(cursor.x, cursor.y)) {
                if (button == GLFW_MOUSE_BUTTON_MIDDLE)
                    ctx->globalControls.beginPan(cursor.x, cursor.y);
                else
                    ctx->globalControls.beginOrbit(cursor.x, cursor.y);
            }
        } else if (action == GLFW_RELEASE) {
            ctx->globalControls.end();
        }
        return;
    }

    if (sim->currentState)
        sim->currentState->handleMouseButton(*sim, *ctx->renderer,
                                                  window, button, action);
}

// Scroll wheel zooms the global camera while the cursor is over its viewport.
void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    (void)xoffset;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim)
        return;

    const glm::dvec2 cursor = cursorPosPixels(window);
    if (ctx->sim->globalView.viewport.contains(cursor.x, cursor.y))
        ctx->globalControls.zoomBy(ctx->sim->globalView.camera, yoffset);
}

// Cursor motion pans (middle-drag) or orbits (right-drag) the global camera.
// Fires constantly, so it stays silent and does nothing unless a drag is live.
// The xpos/ypos arguments are screen coords; re-read in pixels so the deltas
// match the drag anchors beginPan/beginOrbit stored.
void cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    (void)xpos; (void)ypos;

    CallbackContext *ctx = static_cast<CallbackContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->sim || !ctx->globalControls.dragging())
        return;

    const glm::dvec2 cursor = cursorPosPixels(window);
    ctx->globalControls.drag(ctx->sim->globalView.camera, cursor.x, cursor.y);
}
