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
};

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);