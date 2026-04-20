#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

// #include "core/AppState.h"
#include "loader/TerrainLoader.h"
#include "render/Renderer.h"
#include "input/Callbacks.h"
#include <GL/glu.h>

AppState appState;

GLFWwindow *initGL() {

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }

    GLFWwindow *window = glfwCreateWindow(800, 600, "OpenGL Window", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, &appState);
    glfwSetKeyCallback(window, keyCallback);
    // glfwSetMouseButtonCallback(window, mouseButtonCallback);
    // glfwSetCursorPosCallback(window, cursorPosCallback);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);                   // Black Background
    glClearDepth(1.0f);                         // Depth Buffer Setup
    glDepthFunc(GL_LEQUAL);                         // Type Of Depth Testing
    glEnable(GL_DEPTH_TEST);                        // Enable Depth Testing
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);          // Enable Alpha Blending (disable alpha testing)
    glEnable(GL_BLEND);                         // Enable Blending       (disable alpha testing)
    // glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping

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

    while(!glfwWindowShouldClose(window)) {
        // Clear the screen and set up for drawing
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Renders the global view camera on left side of the window
        setupCamera(appState.globalCamera);
        renderTerrain(appState.mesh);
        renderGlobalOverlay(appState);
    
        // Renders the local view camera on right side of the window
        setupCamera(appState.playerCamera);
        renderTerrain(appState.mesh);
        renderPlayerOverlay(appState);
    
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwTerminate();

    return 0;
}