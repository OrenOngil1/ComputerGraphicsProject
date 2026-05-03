#include "RecordInput.h"

#include "Movement.h"
#include "../core/Recording.h"
#include <iostream>

void handleKeyRecord(AppState &appState, int key, int mods)
{
    if(key == GLFW_KEY_B)
        recordCamera(appState.cameraRecords, appState.playerCamera);
}