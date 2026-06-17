#include "Window.h"

#include <iostream>
#include <stdexcept>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

// GLFW's global error callback: logs the code + description for any GLFW error.
// Installed only AFTER the checked bring-up sequence below (which throws its own
// self-describing exceptions), so it never double-reports an init failure -- it is
// the catch-all for AMBIENT errors during the running session, from calls whose
// return value we don't inspect (e.g. event-loop-time errors). The description is
// owned by GLFW and only valid during this call, so it is logged now, not stored.
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
// Any failure throws: a constructor can't return nullptr, and a half-built window
// is not a usable object.
Window::Window(int width, int height, const char *title)
{
    // The cause of the last GLFW error (code + description), formatted for an
    // exception message. Bound local to the ctor. glfwGetError is pre-init-safe --
    // it captures even a glfwInit failure -- so reading it right AT a failing call
    // makes the thrown exception self-describing, not a generic "it failed". The
    // error callback is installed only after this sequence (see below), so these
    // init failures are reported once, by the exception alone -- never doubled.
    auto glfwError = [] {
        const char *desc = nullptr;
        int code = glfwGetError(&desc);
        return "(" + std::to_string(code) + ") " + (desc ? desc : "no description");
    };

    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW: " + glfwError());

    // Hints must be set BEFORE glfwCreateWindow -- they configure the context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window: " + glfwError());
    }

    glfwMakeContextCurrent(m_window);

    // GLAD looks up every gl* function pointer the driver provides. Calling any
    // gl* function before this returns is a null-pointer crash.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
        throw std::runtime_error("Failed to load OpenGL via GLAD");
    }

    // Bring-up is past its checked, self-describing-exception phase: install the
    // global error callback now so any AMBIENT error during the running session is
    // logged, without it ever double-reporting an init failure handled above.
    glfwSetErrorCallback(glfwErrorCallback);

    // Renderer + vendor alongside the version: under WSLg this is how you tell a
    // GPU-backed context ("D3D12 (NVIDIA ...)") from a software fallback
    // ("llvmpipe"), which is also what decides whether the compositor shows the
    // window in its slow "COPY MODE" path.
    std::cout << "OpenGL Version:  " << glGetString(GL_VERSION)  << std::endl;
    std::cout << "OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "OpenGL Vendor:   " << glGetString(GL_VENDOR)   << std::endl;
}

Window::~Window()
{
    // glfwTerminate destroys all remaining windows and frees the library's
    // resources. It must run AFTER every GL object is deleted (see header) --
    // member-init order in Application guarantees that.
    glfwTerminate();
}
