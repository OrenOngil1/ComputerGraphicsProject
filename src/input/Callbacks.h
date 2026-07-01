#pragma once

#include <GLFW/glfw3.h>

#include "CameraControls.h"   // OrbitController, MovementIntent

struct Simulation;
class Renderer;

struct CallbackContext {
    Simulation *sim = nullptr;
    Renderer *renderer = nullptr;
    OrbitController globalControls;
};

// GLFW glue that gathers the currently-held movement keys into a device-neutral intent.
// Declared here (input layer) so the States can call it each frame; defined in
// Callbacks.cpp to keep GLFW out of the pure CameraControls TU.
MovementIntent pollMovementIntent(GLFWwindow *window);

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

// Global-map view controls: scroll zooms, middle-drag pans -- both move only the
// global (overview) camera, to make color-picking 3D points there easier.
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);