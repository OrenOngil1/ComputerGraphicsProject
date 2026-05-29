#pragma once

#include <glm/glm.hpp>

// A camera answers "what does the world look like from here?" -- purely the
// eye's pose and lens. It carries no notion of where on screen it is drawn;
// that is the Viewport's job. Keeping these apart lets the camera be passed
// as read-only (const&) data: the renderer never has to mutate it.
struct Camera {
    glm::vec3 position;
    glm::vec3 target;
    glm::vec3 up;

    float fov;
    float near;
    float far;
};

// A viewport answers "which rectangle of the window do I paint into?" -- pure
// render configuration. It is derived from the window size each frame (see
// leftHalf/rightHalf in Renderer.h), so nothing needs to store it.
struct Viewport {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// For recording camera positions and targets for later playback
struct CameraRecord {
    glm::vec3 position;
    glm::vec3 target;
};