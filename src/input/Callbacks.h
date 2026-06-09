#pragma once

#include <GLFW/glfw3.h>

// Non-owning bundle of the collaborators a GLFW callback needs. The window's single
// user pointer points at one of these; main owns the actual AppState and Renderer on
// the stack (so GPU resources still die before glfwTerminate) -- AppContext only refers
// to them. Lets a callback reach a service (Renderer) without a global/singleton.
struct AppState;
class Renderer;

struct AppContext {
    AppState *appState = nullptr;
    Renderer *renderer = nullptr;
};

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

// void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);