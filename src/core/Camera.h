#pragma once

#include <glm/glm.hpp>

typedef struct {

    // Viewport parameters
    int x;
    int y;
    int width;
    int height;
    
    // Camera parameters
    glm::vec3 position;
    glm::vec3 target;
    glm::vec3 up;

    // Projection parameters
    float fov;
    float near;
    float far;
    
} Camera;

// For recording camera positions and targets for later playback
typedef struct {
    glm::vec3 position;
    glm::vec3 target;
} CameraRecord;