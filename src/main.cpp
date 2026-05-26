#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "loader/TerrainLoader.h"
#include "render/Renderer.h"
#include "input/Callbacks.h"

AppState appState;

GLFWwindow *initGL() {

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(800, 600, "OpenGL Window", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to load OpenGL via GLAD" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    glfwSetWindowUserPointer(window, &appState);
    glfwSetKeyCallback(window, keyCallback);

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
    std::string image_path = "assets/terrain1.jpg";

    appState.mesh = readTerrain(image_path);
    appState.terrainSize = std::max(appState.mesh.width, appState.mesh.height);

    GLFWwindow *window = initGL();
    if (!window) {
        std::cerr << "Failed to initialize OpenGL" << std::endl;
        return -1;
    }

    int windowWidth;
    glfwGetWindowSize(window, &windowWidth, NULL);

    // global camera will be above the terrain
    // looking towards the center
    appState.globalCamera = {
        .x        = 0,
        .y        = 0,
        .position = glm::vec3(0.0f, appState.terrainSize * 0.8f, appState.terrainSize * 1.4f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = appState.terrainSize * 3.0f
    };

    // player camera will be controlled by the user
    // starting at the edge of the terrain, looking towards the center
    appState.playerCamera = {
        .x        = windowWidth / 2,
        .y        = 0,
        .position = glm::vec3(0.0f, appState.terrainSize * 0.07f, appState.terrainSize * 0.5f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = appState.terrainSize * 3.0f
    };

    Shader sceneShader("assets/shaders/terrain.shader");
    TerrainGpu terrainGpu = uploadTerrain(appState.mesh);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Renders the global view camera on left side of the window
        setupCamera(appState.globalCamera);
        {
            glm::mat4 mvp = computeViewProjection(appState.globalCamera);
            renderTerrain(terrainGpu, sceneShader, mvp);
            renderGlobalOverlay(appState, sceneShader, mvp);
        }

        // Renders the local view camera on right side of the window
        setupCamera(appState.playerCamera);
        {
            glm::mat4 mvp = computeViewProjection(appState.playerCamera);
            renderTerrain(terrainGpu, sceneShader, mvp);
            renderPlayerOverlay(appState, sceneShader, mvp);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();

    return 0;
}
