#include <iostream>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "loader/TerrainLoader.h"
#include "render/Renderer.h"
#include "input/Callbacks.h"
#include "state/States.h"

#define WIDTH 800
#define HEIGHT 600

// Init order is load-bearing here:
//   1. glfwInit                       -- start the window system
//   2. glfwWindowHint(...)             -- request a 3.3 core context BEFORE create
//   3. glfwCreateWindow                -- the context is born with the requested version
//   4. glfwMakeContextCurrent          -- bind the context to this thread
//   5. gladLoadGLLoader                -- look up gl* function pointers; nothing
//                                         can call any gl* function before this
//   6. glClearColor / glEnable / etc.  -- now safe
GLFWwindow *initGL(AppState &appState) {

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }

    // Hints must be set BEFORE glfwCreateWindow -- they configure the context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "OpenGL Window", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);

    // GLAD looks up every gl* function pointer the driver provides. Calling any
    // gl* function before this returns is a null-pointer crash.
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to load OpenGL via GLAD" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    glfwSetWindowUserPointer(window, &appState);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Seed the initial split-screen layout. The framebuffer-size callback only
    // fires on a CHANGE, never at startup, so we invoke it once by hand. The size
    // comes from glfwGetFramebufferSize -- the actual size in PIXELS -- not the
    // WIDTH/HEIGHT window-request macros, which are screen coordinates the window
    // manager / HiDPI scaling may not match.
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    framebufferSizeCallback(window, fbWidth, fbHeight);

    // Persistent GL state -- set once, applies to every draw call afterward.
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    return window;
}

int main()
{
    // The single owner of all application state -- no longer a global. main() is
    // the composition root: it constructs appState, hands it to initGL (which wires
    // it into the GLFW window via the user pointer), and tears it down on return.
    AppState appState;

    std::string image_path = "assets/terrain1.jpg";

    appState.mesh = readTerrain(image_path);
    appState.terrainSize = std::max(appState.mesh.width, appState.mesh.height);

    GLFWwindow *window = initGL(appState);
    if (!window) {
        std::cerr << "Failed to initialize OpenGL" << std::endl;
        return -1;
    }

    // global camera will be above the terrain
    // looking towards the center
    appState.globalView.camera = {
        .position = glm::vec3(0.0f, appState.terrainSize * 0.8f, appState.terrainSize * 1.4f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = appState.terrainSize * 3.0f
    };

    // player camera will be controlled by the user
    // starting at the edge of the terrain, looking towards the center
    appState.playerView.camera = {
        .position = glm::vec3(0.0f, appState.terrainSize * 0.07f, appState.terrainSize * 0.5f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = appState.terrainSize * 3.0f
    };

    // Start in free-navigation mode. Must be set before the loop: both
    // keyCallback and Renderer::renderView dereference currentState.
    appState.currentState = std::make_unique<NavigationState>();

    // Scope ensures the Renderer's GPU resources (Shader, TerrainGpu) are
    // destroyed while the OpenGL context is still alive. glfwTerminate() below
    // tears down the context, so any gl* call after it (including the Renderer's
    // destructor) would be a null-ptr crash.
    {
        Renderer renderer(appState.mesh);

        while (!glfwWindowShouldClose(window)) {
            renderer.clear();

            // The split-screen layout is maintained by framebufferSizeCallback
            // (recomputed once per resize), so the loop just draws each stored View.
            renderer.renderGlobalView(appState.globalView, appState);
            renderer.renderPlayerView(appState.playerView, appState);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    } // Renderer destroyed here (glDeleteProgram/glDeleteBuffers) — context still alive

    glfwTerminate();

    return 0;
}
