#pragma once

#include "Scene.h"
#include "Camera.h"

enum class Mode { NONE, RECORD, PLAYBACK, PICK, TRACKERS, FEATURE_DETECTION_2D };

struct AppState {
    Mode mode = Mode::NONE;
    float terrainSize = 0.0f;
    size_t playbackIndex = 0;
    Mesh mesh;
    Camera globalCamera;
    Camera playerCamera;
    std::vector<glm::vec3> pathPoints;
    std::vector<CameraRecord> cameraRecords;
};

extern AppState appState;