#include "Callbacks.h"

#include <iostream>
#include <memory>

#include "../core/Simulation.h"
#include "../state/States.h"

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

    // A global mode-switch hotkey takes precedence over in-mode handling.
    if (tryTransition(*sim, key, mods))
        return;

    if (sim->currentState)
        sim->currentState->handleKey(*sim, key, mods);
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
    if (sim->currentState)
        sim->currentState->handleMouseButton(*sim, *ctx->renderer,
                                                  window, button, action);
}
