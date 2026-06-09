#include "Callbacks.h"

#include <iostream>
#include <memory>

#include "../core/AppState.h"
#include "../state/States.h"

// The one place a transition is performed -- so a state never has to know about
// the states it can transition to -- and the new mode's entry action (onEnter)
// runs in exactly one spot, right after it becomes current.
static void setState(AppState &appState, std::unique_ptr<State> next)
{
    appState.currentState = std::move(next);
    appState.currentState->onEnter(appState);
}

// PLAYBACK and PICK both require at least one recorded waypoint. The precondition
// is a property of the transition (you may only enter if waypoints exist), so it
// is checked here, before the swap is committed -- not in onEnter, which runs
// after the swap and so would be too late to refuse.
static bool requireWaypoints(const AppState &appState)
{
    if (appState.waypoints.empty()) {
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
static bool tryTransition(AppState &appState, int key, int mods)
{
    switch (key) {
        case GLFW_KEY_R:
            if (mods & GLFW_MOD_CONTROL) {
                if (!requireWaypoints(appState))
                    return true;
                setState(appState, std::make_unique<PlaybackState>());
                std::cout << "Switched to PLAYBACK mode" << std::endl;
            } else {
                setState(appState, std::make_unique<RecordState>());
                std::cout << "Switched to RECORD mode" << std::endl;
            }
            return true;

        case GLFW_KEY_P:
            if (!requireWaypoints(appState))
                return true;
            setState(appState, std::make_unique<PickState>());
            std::cout << "Switched to PICK mode" << std::endl;
            return true;
    }

    return false;
}

// On resize GLFW hands us the new framebuffer size in PIXELS (the units
// glViewport wants -- unlike window size, which is screen coordinates that
// differ from pixels under HiDPI scaling). We recompute the split-screen layout
// here, once per resize, and store each viewport in its View; the render loop
// just reads them. Reaches AppState through the window user pointer, exactly
// like keyCallback below.
void framebufferSizeCallback(GLFWwindow *window, int width, int height)
{
    AppContext *ctx = static_cast<AppContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->appState) {
        std::cerr << "Error: No AppContext associated with window" << std::endl;
        return;
    }
    AppState *appState = ctx->appState;

    appState->globalView.viewport = leftHalf(width, height);
    appState->playerView.viewport = rightHalf(width, height);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    (void)scancode;

    AppContext *ctx = static_cast<AppContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->appState) {
        std::cerr << "Error: No AppContext associated with window" << std::endl;
        return;
    }
    AppState *appState = ctx->appState;

    // Only handle key presses and repeats, ignore releases.
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    // Escape closes the window, which ends the render loop and returns to the
    // terrain menu (the outer loop in main re-enters it).
    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, true);
        return;
    }

    // A global mode-switch hotkey takes precedence over in-mode handling.
    if (tryTransition(*appState, key, mods))
        return;

    if (appState->currentState)
        appState->currentState->handleKey(*appState, key, mods);
}

// Mouse buttons are routed to the active state unconditionally; the state decides
// whether it cares (only PickState reacts, to a left-click). The Renderer is handed
// through so PICK can run its color-pick render pass in response.
void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    (void)mods;

    AppContext *ctx = static_cast<AppContext *>(glfwGetWindowUserPointer(window));
    if (!ctx || !ctx->appState) {
        std::cerr << "Error: No AppContext associated with window" << std::endl;
        return;
    }

    AppState *appState = ctx->appState;
    if (appState->currentState)
        appState->currentState->handleMouseButton(*appState, *ctx->renderer,
                                                  window, button, action);
}
