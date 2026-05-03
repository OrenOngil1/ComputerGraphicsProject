#pragma once

#include "Scene.h"
#include "Camera.h"

#include <memory>

#define NKEYS 1024

enum class Mode { NONE, RECORD, PLAYBACK, PICK, TRACKERS, FEATURE_DETECTION_2D };

struct AppState {
    Mode mode = Mode::NONE;
    bool isFreeMovement = true;
    float terrainSize = 0.0f;
    size_t playbackIndex = 0;
    bool keys[NKEYS] = { false };
    Mesh mesh;
    Camera globalCamera;
    Camera playerCamera;
    std::unique_ptr<CameraRecord> computedCameraFromPicking = nullptr;
    std::vector<glm::vec3> pathPoints;
    std::vector<CameraRecord> cameraRecords;
    std::vector<PickedPoint> pickedPoints;
    
};