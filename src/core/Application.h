#pragma once

#include <string>

#include "Window.h"
#include "Simulation.h"
#include "../render/Renderer.h"

// The composition root, as an object. Owns the platform window, the simulation
// state, and the GPU renderer for one whole program run. The non-owning
// CallbackContext that callbacks read off the GLFW user pointer is a *local in
// run()* (its lifetime matches the session loop), so Application holds no pointer
// into its own members -- no self-reference.
//
// Member DECLARATION ORDER is load-bearing: m_window brings the GL context up
// first (so m_renderer can compile its shaders), and reverse-order destruction
// runs ~Renderer (glDelete*) before ~Window (glfwTerminate). That encodes, in the
// type, the teardown ordering main() used to guard with a manual { } scope.
class Application {
public:
    Application();   // wire callbacks + configure GL state (the user pointer is set in run())
    int run();       // the menu/session loop; returns the process exit code

    // Owns a Window + Renderer (GLFW/GL resources, non-copyable), so an Application
    // is non-copyable too. It holds no pointer into its own members, so this is pure
    // resource ownership -- not a self-reference workaround.
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

private:
    bool loadTerrain(const std::string &path);   // read mesh, swap it in, reset per-terrain state; false => load failed
    void runSession();                            // one terrain's frame loop, until Escape/close

    Window     m_window;     // (1) GL context goes live           -- destroyed LAST
    Simulation m_sim;        // (2)
    Renderer   m_renderer;   // (3) compiles shaders (context live) -- destroyed BEFORE Window
};
