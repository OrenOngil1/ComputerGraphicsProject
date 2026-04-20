#include "PlaybackInput.h"

enum class Sign { PLUS = 1, MINUS = -1 };

void toggleRecords(const std::vector<CameraRecord> &cameraRecords, size_t &playbackIndex, Camera &playerCamera, Sign sign)
{
    if(cameraRecords.empty()) return;

    const auto& record = cameraRecords[playbackIndex];

    playerCamera.position = record.position;
    playerCamera.target = record.target;

    int size = static_cast<int>(cameraRecords.size());
    playbackIndex = (playbackIndex + static_cast<int>(sign) + size) % size;
}

void handleKeyPlayback(AppState &appState, int key)
{
    if(key == GLFW_KEY_UP)
        toggleRecords(appState.cameraRecords, appState.playbackIndex, appState.playerCamera, Sign::PLUS);
    else if(key == GLFW_KEY_DOWN)
        toggleRecords(appState.cameraRecords, appState.playbackIndex, appState.playerCamera, Sign::MINUS);
}