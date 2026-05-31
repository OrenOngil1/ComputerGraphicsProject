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

// PLAYBACK and PICK both require at least one recorded camera. The precondition
// is a property of the transition (you may only enter if records exist), so it
// is checked here, before the swap is committed -- not in onEnter, which runs
// after the swap and so would be too late to refuse.
static bool requireRecords(const AppState &appState)
{
    if (appState.cameraRecords.empty()) {
        std::cout << "Record camera positions in RECORD mode first" << std::endl;
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
                if (!requireRecords(appState))
                    return true;
                setState(appState, std::make_unique<PlaybackState>());
                std::cout << "Switched to PLAYBACK mode" << std::endl;
            } else {
                setState(appState, std::make_unique<RecordState>());
                std::cout << "Switched to RECORD mode" << std::endl;
            }
            return true;

        case GLFW_KEY_P:
            if (!requireRecords(appState))
                return true;
            setState(appState, std::make_unique<PickState>());
            std::cout << "Switched to PICK mode" << std::endl;
            return true;
    }

    return false;
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    (void)scancode;

    AppState *appState = static_cast<AppState *>(glfwGetWindowUserPointer(window));
    if (!appState) {
        std::cerr << "Error: No AppState associated with window" << std::endl;
        return;
    }

    // Only handle key presses and repeats, ignore releases.
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    // A global mode-switch hotkey takes precedence over in-mode handling.
    if (tryTransition(*appState, key, mods))
        return;

    if (appState->currentState)
        appState->currentState->handleKey(*appState, key, mods);
}
