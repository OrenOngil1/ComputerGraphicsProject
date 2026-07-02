#include "Application.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Menu.h"
#include "../loader/TerrainLoader.h"
#include "../input/Callbacks.h"
#include "../state/States.h"

namespace {

constexpr int WIDTH  = 800;
constexpr int HEIGHT = 600;

// Persistent GL state -- set once, applies to every draw call afterward.
void configureGLState()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
}

// Place the two cameras relative to the terrain: the global camera looks down
// at the whole terrain, the player camera starts low at the edge looking in.
// Both scale with terrainSize so any DEM frames sensibly.
void configureViews(Simulation &sim)
{
    sim.globalView.camera = {
        glm::vec3(0.0f, sim.terrainSize * 0.8f, sim.terrainSize * 1.4f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f,
        0.1f,
        sim.terrainSize * 3.0f
    };

    sim.playerView.camera = {
        glm::vec3(0.0f, sim.terrainSize * 0.07f, sim.terrainSize * 0.5f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        45.0f,
        0.1f,
        sim.terrainSize * 3.0f
    };
}

} // namespace

Application::Application()
    : m_window(WIDTH, HEIGHT, "OpenGL Window")
{
    GLFWwindow *window = m_window.handle();

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetScrollCallback(window, scrollCallback);          // global-map zoom
    glfwSetCursorPosCallback(window, cursorPosCallback);    // global-map pan/orbit drag

    // Seed the split-screen layout from the framebuffer size in PIXELS -- the
    // requested WIDTH/HEIGHT are screen coords, which HiDPI scaling may not
    // match. Same layout helpers the resize callback uses for later resizes.
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    m_sim.globalView.viewport = leftHalf(fbWidth, fbHeight);
    m_sim.playerView.viewport = rightHalf(fbWidth, fbHeight);

    configureGLState();
}

bool Application::loadTerrain(const std::string &path)
{
    std::optional<Mesh> mesh = readTerrain(path);
    if (!mesh)
        return false;

    m_sim.mesh = std::move(*mesh);
    m_sim.terrainSize = static_cast<float>(std::max(m_sim.mesh.cols, m_sim.mesh.rows));

    m_renderer.loadTerrain(m_sim.mesh);

    configureViews(m_sim);

    // m_sim outlives the terrain, so the "fresh per terrain" reset is explicit:
    // recordings must not bleed from one terrain into the next.
    m_sim.pathPoints.clear();
    m_sim.waypoints.clear();
    m_sim.currentState = std::make_unique<NavigationState>();
    return true;
}

void Application::runSession()
{
    GLFWwindow *window = m_window.handle();
    m_sim.returnToMenu = false;

    // Seed `last` before the loop so the first dt is tiny, not since-epoch.
    float last = (float)glfwGetTime();

    while (!glfwWindowShouldClose(window) && !m_sim.returnToMenu) {
        const float now = (float)glfwGetTime();
        const float dt = now - last;   // seconds; makes motion frame-rate independent
        last = now;

        // The active mode advances itself (the moving modes poll held keys).
        if (m_sim.currentState)
            m_sim.currentState->tick(m_sim, window, dt);

        m_renderer.clear();
        m_renderer.renderGlobalView(m_sim.globalView, m_sim);
        m_renderer.renderPlayerView(m_sim.playerView, m_sim);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

int Application::run()
{
    GLFWwindow *window = m_window.handle();

    // The callback bundle lives on run()'s stack: callbacks only fire during
    // the session loop, so its lifetime is exactly run().
    CallbackContext context{ &m_sim, &m_renderer };
    glfwSetWindowUserPointer(window, &context);

    while (true) {
        std::optional<std::string> path = selectTerrain("assets/terrains/");
        if (!path)
            break;   // menu "Exit": unwind normally, destructors run

        if (!loadTerrain(*path)) {
            std::cerr << "Could not load that terrain -- pick another." << std::endl;
            continue;
        }

        runSession();

        // Escape (returnToMenu) loops back to the menu; Ctrl+Q or the OS close
        // button trip glfwWindowShouldClose and quit.
        if (glfwWindowShouldClose(window))
            break;
    }
    return 0;
}
