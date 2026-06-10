#include "Application.h"

#include <algorithm>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Menu.h"
#include "../loader/TerrainLoader.h"
#include "../input/Callbacks.h"
#include "../state/States.h"

namespace {

constexpr int WIDTH  = 800;
constexpr int HEIGHT = 600;

// Persistent GL state -- set once, applies to every draw call afterward. The app
// half of the old initGL(); needs only a live context, which the Window provides.
void configureGLState()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
}

// Scene data, not control flow: place the two cameras relative to the terrain.
// The global camera looks down at the whole terrain; the player camera starts low
// at the edge looking in. Both scale with terrainSize so any DEM frames sensibly.
void configureViews(Simulation &sim)
{
    sim.globalView.camera = {
        .position = glm::vec3(0.0f, sim.terrainSize * 0.8f, sim.terrainSize * 1.4f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = sim.terrainSize * 3.0f
    };

    sim.playerView.camera = {
        .position = glm::vec3(0.0f, sim.terrainSize * 0.07f, sim.terrainSize * 0.5f),
        .target   = glm::vec3(0.0f, 0.0f, 0.0f),
        .up       = glm::vec3(0.0f, 1.0f, 0.0f),
        .fov      = 45.0f,
        .near     = 0.1f,
        .far      = sim.terrainSize * 3.0f
    };
}

} // namespace

// Window constructs first (member-init order), so the GL context is live here for
// callback registration and the persistent GL state. The GLFW user pointer is NOT
// set here -- it points at a CallbackContext that lives in run() (its lifetime is
// the session loop), and the callbacks' null-pointer guards cover the brief gap
// until run() wires it.
Application::Application()
    : m_window(WIDTH, HEIGHT, "OpenGL Window")
{
    GLFWwindow *window = m_window.handle();

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Seed the split-screen layout from the actual framebuffer size in PIXELS (the
    // requested WIDTH/HEIGHT are screen coords HiDPI scaling may not match). We call
    // the same layout helpers the resize callback uses, directly -- so the seed needs
    // no user pointer and runs here, before run() wires one. The callback itself only
    // matters for *later* resizes.
    int fbWidth, fbHeight;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    m_sim.globalView.viewport = leftHalf(fbWidth, fbHeight);
    m_sim.playerView.viewport = rightHalf(fbWidth, fbHeight);

    configureGLState();
}

// Swap to a new terrain in place: upload the mesh (shaders untouched), reposition
// the cameras, and reset the per-terrain state. The last step matters because
// m_sim is now long-lived -- the "fresh per terrain" guarantee the old
// in-loop local gave for free must be made explicit here.
void Application::loadTerrain(const std::string &path)
{
    m_sim.mesh = readTerrain(path);
    m_sim.terrainSize = std::max(m_sim.mesh.width, m_sim.mesh.height);

    m_renderer.loadTerrain(m_sim.mesh);

    configureViews(m_sim);

    m_sim.pathPoints.clear();
    m_sim.waypoints.clear();
    m_sim.currentState = std::make_unique<NavigationState>();
}

// One terrain's frame loop. Continuous movement integrates over the time since the
// last frame, so motion is frame-rate independent; seed `last` before the loop so
// the first dt is tiny. Exits on either signal: OS close or Escape (returnToMenu).
void Application::runSession()
{
    GLFWwindow *window = m_window.handle();
    m_sim.returnToMenu = false;

    float last = (float)glfwGetTime();

    while (!glfwWindowShouldClose(window) && !m_sim.returnToMenu) {
        const float now = (float)glfwGetTime();
        const float dt = now - last;
        last = now;

        // The active mode advances itself (the moving modes poll held keys to fly
        // the player camera; Playback/Pick inherit a no-op tick).
        if (m_sim.currentState)
            m_sim.currentState->tick(m_sim, window, dt);

        m_renderer.clear();
        m_renderer.renderGlobalView(m_sim.globalView, m_sim);
        m_renderer.renderPlayerView(m_sim.playerView, m_sim);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

// Menu/session loop. Each pass picks a terrain and runs it until a signal ends the
// session. Escape (returnToMenu) loops back to the menu; the OS close button quits.
// selectTerrain's "Exit" entry (exit(0)) is the other way out of the program.
int Application::run()
{
    GLFWwindow *window = m_window.handle();

    // The callback bundle lives on run()'s stack: callbacks only fire during the
    // session loop, so its lifetime is exactly run(). Pointing the user pointer at a
    // local (not a member) keeps Application free of any pointer into its own fields.
    CallbackContext context{ &m_sim, &m_renderer };
    glfwSetWindowUserPointer(window, &context);

    while (true) {
        loadTerrain(selectTerrain("assets/terrains/"));
        runSession();

        // Esc set returnToMenu -> next menu pass. Ctrl+Q or the OS close button quit
        // (both via glfwWindowShouldClose); the menu's "Exit" is the third way out.
        if (glfwWindowShouldClose(window))
            break;
    }
    return 0;
}
