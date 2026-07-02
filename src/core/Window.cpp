#include "Window.h"

#include <iostream>
#include <stdexcept>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Catch-all for ambient GLFW errors during the running session (calls whose
// return value isn't inspected, e.g. at event-loop time). Installed only after
// the checked bring-up below, so an init failure is never reported twice. The
// description is owned by GLFW and only valid during this call.
static void glfwErrorCallback(int code, const char *description)
{
    std::cerr << "GLFW error " << code << ": " << description << std::endl;
}

// Init order is load-bearing:
//   1. glfwInit                -- start the window system
//   2. glfwWindowHint          -- request a 3.3 core context BEFORE create
//   3. glfwCreateWindow        -- the context is born with the requested version
//   4. glfwMakeContextCurrent  -- bind the context to this thread
//   5. gladLoadGLLoader        -- look up gl* pointers; nothing may call gl* before this
// Any failure throws: a half-built window is not a usable object.
Window::Window(int width, int height, const char *title)
{
    // The last GLFW error (code + description) formatted for an exception
    // message. glfwGetError is pre-init-safe, so it captures even a glfwInit
    // failure.
    auto glfwError = [] {
        const char *desc = nullptr;
        int code = glfwGetError(&desc);
        return "(" + std::to_string(code) + ") " + (desc ? desc : "no description");
    };

    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW: " + glfwError());

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window: " + glfwError());
    }

    glfwMakeContextCurrent(m_window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
        throw std::runtime_error("Failed to load OpenGL via GLAD");
    }

    glfwSetErrorCallback(glfwErrorCallback);

    // Renderer/vendor tell a GPU-backed context from a software fallback
    // (e.g. "llvmpipe" under WSLg).
    std::cout << "OpenGL Version:  " << glGetString(GL_VERSION)  << std::endl;
    std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "OpenGL Vendor:   " << glGetString(GL_VENDOR)   << std::endl;
}

Window::~Window()
{
    // Destroys all remaining windows and frees the library's resources.
    glfwTerminate();
}
