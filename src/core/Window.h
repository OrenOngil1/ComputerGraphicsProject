#pragma once

struct GLFWwindow;

// RAII owner of the GLFW library + the single window/context. Construction
// brings the platform layer up (glfwInit -> 3.3-core window -> GLAD);
// destruction tears it down (glfwTerminate, which must run after every GL
// object is deleted -- Application's member order guarantees that).
class Window {
public:
    Window(int width, int height, const char *title);   // throws on any init failure
    ~Window();

    // Non-copyable: owns a process-global resource (the GLFW library).
    Window(const Window &) = delete;
    Window &operator=(const Window &) = delete;

    GLFWwindow *handle() const { return m_window; }

private:
    GLFWwindow *m_window = nullptr;
};
