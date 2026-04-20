#include "Callbacks.h"

#include <iostream>

#include "RecordInput.h"
#include "PlaybackInput.h"
#include "Movement.h"

int changeMode(AppState &appState, int key, int mods)
{
    switch (key) {

        case GLFW_KEY_R:
        
            if (mods & GLFW_MOD_CONTROL) {
                appState.mode = Mode::PLAYBACK;
                std::cout << "Switched to PLAYBACK mode" << std::endl;
            } else {
                appState.mode = Mode::RECORD;
                std::cout << "Switched to RECORD mode" << std::endl;
            }   

            return 1;

        case GLFW_KEY_P:

            appState.mode = Mode::PICK;
            appState.pathPoints.clear();
            std::cout << "Switched to PICK mode" << std::endl;
            
            return 1;
    }

    return 0;
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    AppState *appState = static_cast<AppState *>(glfwGetWindowUserPointer(window));
    if(!appState) {
        std::cerr << "Error: No AppState associated with window" << std::endl;
        return;
    }

    // Only handle key presses and repeats, ignore releases
    if(action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    // if we handled a mode change
    // don't handle it as a regular key press in the current mode
    if(changeMode(*appState, key, mods))
        return;

    switch (appState->mode) {
        case Mode::NONE:
            std::cout << "key pressed in NONE mode" << std::endl;
            handleMovement(appState->playerCamera, appState->terrainSize, key, mods);
            break;
        case Mode::RECORD:
            handleKeyRecord(*appState, key, mods);
            break;
        case Mode::PLAYBACK:
            handleKeyPlayback(*appState, key);
            break;
        default:
            break;
    }
}