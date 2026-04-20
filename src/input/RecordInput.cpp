#include "RecordInput.h"

#include "Movement.h"
#include "../core/Recording.h"
#include <iostream>

void handleKeyRecord(AppState &appState, int key, int mods)
{
    glm::vec3 prevPosition = appState.playerCamera.position;
    handleMovement(appState.playerCamera, appState.terrainSize, key, mods);

    // Record the camera position as a path point if it has changed
    if(appState.playerCamera.position != prevPosition)
        recordPathPoint(appState.pathPoints, appState.playerCamera.position);

    if(key == GLFW_KEY_B)
        recordCamera(appState.cameraRecords, appState.playerCamera);
}