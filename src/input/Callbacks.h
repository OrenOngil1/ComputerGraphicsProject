#pragma once

#include <GLFW/glfw3.h>

#include "CameraControls.h"   // OrbitController, MovementIntent

struct Simulation;
class Renderer;

// What the GLFW callbacks reach through the window user pointer. Owned by
// Application::run(); sim/renderer are non-owning.
struct CallbackContext {
    Simulation *sim = nullptr;
    Renderer *renderer = nullptr;
    OrbitController globalControls;   // middle/right-drag state for the global map
};

// Gather the currently-held movement keys into a device-neutral intent.
// Declared here (input layer) so states can poll it each frame; defined in
// Callbacks.cpp to keep GLFW out of the pure CameraControls TU.
MovementIntent pollMovementIntent(GLFWwindow *window);

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

// Global-map view controls: scroll zooms, middle-drag pans, right-drag orbits
// -- all move only the global (overview) camera.
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
