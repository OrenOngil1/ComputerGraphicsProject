#include "Callbacks.h"

#include <iostream>

#include "Movement.h"
#include "RecordInput.h"
#include "PlaybackInput.h"
#include "PickInput.h"
#include "../core/Utils.h"


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

            if(appState.cameraRecords.empty()) {
                std::cout << "No camera positions recorded before picking mode" << std::endl;
                std::cout << "Record some camera positions in RECORD mode first" << std::endl;
                return 1;
            }

            appState.pathPoints.clear();
            appState.mode = Mode::PICK;

            // Randomly position the camera at one of the recorded positions to start picking
            const auto& record = appState.cameraRecords[randomIndex(appState.cameraRecords.size())];
            appState.playerCamera.position = record.position;
            appState.playerCamera.target = record.target;

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

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    AppState *appState = static_cast<AppState *>(glfwGetWindowUserPointer(window));
    if(!appState) {
        std::cerr << "Error: No AppState associated with window" << std::endl;
        return;
    }

    switch (appState->mode) {
        case Mode::NONE:
        case Mode::RECORD:
        case Mode::PLAYBACK:
            return; // No mouse handling in these modes
        case Mode::PICK:
            if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);
                handlePickMouseButton(*appState, xpos, ypos);
            }
            break;
    }
}
