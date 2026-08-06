#include "Application.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <Debugger.h>   // GLCall

#include "Menu.h"
#include "../loader/TerrainLoader.h"
#include "../input/Callbacks.h"
#include "../state/States.h"

namespace {

constexpr int kWidth  = 800;
constexpr int kHeight = 600;

// Shared lens for both cameras; the far plane scales with the terrain so any
// DEM fits inside the frustum.
constexpr float kFovDeg    = 45.0f;   // vertical, degrees
constexpr float kNearPlane = 0.1f;    // world units
constexpr float kFarFactor = 3.0f;    // far plane = terrainSize * this

// Starting eye positions as factors of terrainSize (world x/z are centered on
// the terrain, so 0.5 is the terrain edge).
constexpr float kGlobalEyeHeight  = 0.8f;    // global: high above ...
constexpr float kGlobalEyeSetback = 1.4f;    // ... and pulled back, whole terrain in frame
constexpr float kPlayerEyeHeight  = 0.07f;   // player: low over the surface ...
constexpr float kPlayerEyeSetback = 0.5f;    // ... at the near edge, looking in

// Persistent GL state -- set once, applies to every draw call afterward.
void configureGLState()
{
    GLCall(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
    GLCall(glClearDepth(1.0f));
    GLCall(glDepthFunc(GL_LEQUAL));
    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    GLCall(glEnable(GL_BLEND));
}

// Place the two cameras relative to the terrain: the global camera looks down
// at the whole terrain, the player camera starts low at the edge looking in.
// Both aim at the terrain center (the world origin).
void configureViews(Simulation &sim)
{
    sim.globalView.camera = {
        glm::vec3(0.0f, sim.terrainSize * kGlobalEyeHeight, sim.terrainSize * kGlobalEyeSetback),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        kFovDeg,
        kNearPlane,
        sim.terrainSize * kFarFactor
    };

    sim.playerView.camera = {
        glm::vec3(0.0f, sim.terrainSize * kPlayerEyeHeight, sim.terrainSize * kPlayerEyeSetback),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        kFovDeg,
        kNearPlane,
        sim.terrainSize * kFarFactor
    };
}

} // namespace

Application::Application()
    : m_window(kWidth, kHeight, "Visual SLAMMER")
{
    GLFWwindow *window = m_window.handle();

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetScrollCallback(window, scrollCallback);          // global-map zoom
    glfwSetCursorPosCallback(window, cursorPosCallback);    // global-map pan/orbit drag

    // Seed the split-screen layout from the framebuffer size in PIXELS -- the
    // requested kWidth/kHeight are screen coords, which HiDPI scaling may not
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
    m_sim.terrainFile = path;

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
    float now, dt;
    float last = (float)glfwGetTime();

    while (!glfwWindowShouldClose(window) && !m_sim.returnToMenu) {
        now = (float)glfwGetTime();
        dt = now - last;   // seconds; makes motion frame-rate independent
        last = now;

        // The active mode advances itself (the moving modes poll held keys).
        if (m_sim.currentState)
            m_sim.currentState->tick(m_sim, window, dt);

        // A minimized window reports a 0x0 framebuffer: skip drawing (a zero
        // viewport makes aspect() divide by zero) but keep polling, so the
        // restore event that re-sizes the viewports still arrives.
        const Viewport &viewport = m_sim.playerView.viewport;
        if (viewport.width > 0 && viewport.height > 0) {
            m_renderer.clear();
            m_renderer.renderGlobalView(m_sim.globalView, m_sim);
            m_renderer.renderPlayerView(m_sim.playerView, m_sim);

            glfwSwapBuffers(window);
        }
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
