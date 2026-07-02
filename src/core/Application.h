#pragma once

#include <string>

#include "Window.h"
#include "Simulation.h"
#include "../render/Renderer.h"

// The composition root: owns the platform window, the app state, and the GPU
// renderer for one whole program run.
//
// Member DECLARATION ORDER is load-bearing: m_window brings the GL context up
// first (so m_renderer can compile its shaders), and reverse-order destruction
// runs ~Renderer (glDelete*) before ~Window (glfwTerminate).
class Application {
public:
    Application();   // wire callbacks + configure persistent GL state
    int run();       // the menu/session loop; returns the process exit code

    // Non-copyable: owns GLFW/GL resources.
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

private:
    // Read a mesh, swap it into the renderer, reset per-terrain state.
    // False = load failed, nothing mutated; the caller can retry.
    bool loadTerrain(const std::string &path);
    // One terrain's frame loop, until Escape (menu) or window close (quit).
    void runSession();

    Window     m_window;     // (1) GL context goes live            -- destroyed LAST
    Simulation m_sim;        // (2)
    Renderer   m_renderer;   // (3) compiles shaders (context live) -- destroyed BEFORE Window
};
