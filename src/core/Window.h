#pragma once

struct GLFWwindow;

// RAII owner of the GLFW library + the single window/context. Construction brings
// the whole platform layer up (glfwInit -> 3.3-core window -> GLAD); destruction
// tears it down (glfwTerminate). Declaring a Window member *before* the Renderer
// makes member-init order guarantee the GL context is live when the Renderer
// compiles its shaders, and reverse destruction runs ~Renderer (glDelete*) before
// ~Window (glfwTerminate) -- the ordering main() used to guard with a manual scope.
class Window {
public:
    Window(int width, int height, const char *title);   // throws on any init failure
    ~Window();

    // Owns a process-global resource (the GLFW library); never duplicated.
    Window(const Window &) = delete;
    Window &operator=(const Window &) = delete;

    GLFWwindow *handle() const { return m_window; }

private:
    GLFWwindow *m_window = nullptr;
};
