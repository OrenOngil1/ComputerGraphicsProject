#pragma once

#include <GLFW/glfw3.h>

// Non-owning bundle of the collaborators a GLFW callback needs. The window's single
// user pointer points at one of these; the Application owns the actual Simulation and
// Renderer (as members) -- CallbackContext only refers to them. Lets a callback reach a
// service (Renderer) without a global/singleton.
struct Simulation;
class Renderer;

struct CallbackContext {
    Simulation *sim = nullptr;
    Renderer *renderer = nullptr;

    // Global-map panning: middle-button drag pans the overview camera. Held
    // here (lifetime = the session) so the press/move/release callbacks share it.
    bool   panning  = false;
    double lastPanX = 0.0;
    double lastPanY = 0.0;
};

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

// Global-map view controls: scroll zooms, middle-drag pans -- both move only the
// global (overview) camera, to make color-picking 3D points there easier.
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);