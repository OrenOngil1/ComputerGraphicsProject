#include "Window.h"

#include <iostream>
#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

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
    if (!glfwInit())
        throw std::runtime_error("Failed to initialize GLFW");

    // Hints must be set BEFORE glfwCreateWindow -- they configure the context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_window);

    // GLAD looks up every gl* function pointer the driver provides. Calling any
    // gl* function before this returns is a null-pointer crash.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(m_window);
        glfwTerminate();
        throw std::runtime_error("Failed to load OpenGL via GLAD");
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
}

Window::~Window()
{
    // glfwTerminate destroys all remaining windows and frees the library's
    // resources. It must run AFTER every GL object is deleted (see header) --
    // member-init order in Application guarantees that.
    glfwTerminate();
}
