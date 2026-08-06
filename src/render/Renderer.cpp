#include "Renderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "PickEncoding.h"
#include "../state/State.h"
#include "../state/OverlayStyle.h"

static void setupViewport(const Viewport &viewport)
{
    GLCall(glViewport(viewport.x, viewport.y, viewport.width, viewport.height));
}

// How far to draw the player camera's view cone: out to where its center ray
// meets the terrain, so the cone visibly lands on the patch of map the player
// view is showing. A ray that misses (aimed at the sky or off the edge) has no
// such distance, and falls back to a terrain-sized reach, which still
// communicates the direction.
static float viewConeReach(const Simulation &sim)
{
    const Camera &camera = sim.playerView.camera;
    const glm::vec3 direction = glm::normalize(camera.target - camera.position);

    // Fine enough to land on the right hillside, coarse enough to stay cheap:
    // this runs once per frame.
    const float step = std::max(sim.terrainSize * 0.004f, 0.05f);
    if (const std::optional<float> hit = raycastTerrain(sim.mesh, camera.position,
                                                        direction, sim.terrainSize * 2.0f, step))
        return std::max(*hit, sim.terrainSize * 0.02f);   // never collapse to a dot up close

    return sim.terrainSize * 0.75f;
}

// Leaves the shader bound on return: callers reset their uniform state (u_Lit,
// u_UseOverride) through it first, then unbind it themselves.
static void drawMesh(const GpuMesh &gpu, Shader &shader, const glm::mat4 &mvp)
{
    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    gpu.va->Bind();
    gpu.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr));
    gpu.va->Unbind();
    gpu.ib->Unbind();
}

// ── Renderer ──────────────────────────────────────────────────

Renderer::Renderer()
    : m_sceneShader("assets/shaders/terrainShader.glsl"),
      m_pickShader("assets/shaders/pickShader.glsl"),
      m_pointShader("assets/shaders/pointShader.glsl"),
      m_sky("assets/skyboxes/")
{
    // m_terrain stays empty until the first loadTerrain; the tracker sphere is
    // terrain-independent, so it is built once here.
    m_sphere = buildSphereMesh(16, 24);
}

void Renderer::clear() const
{
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
}

void Renderer::loadTerrain(const Mesh &terrain)
{
    m_terrain = uploadTerrain(terrain);   // old buffers freed by the move-assign
}

void Renderer::applyLighting(const DirectionalLight &light)
{
    // Locals, not temporaries: SetUniform4f takes a non-const glm::vec4 &.
    glm::vec4 lightDir(light.direction, 0.0f);
    glm::vec4 lightColor(light.color, 1.0f);
    m_sceneShader.Bind();
    m_sceneShader.SetUniform1i("u_Lit", 1);
    m_sceneShader.SetUniform4f("u_LightDir", lightDir);
    m_sceneShader.SetUniform4f("u_LightColor", lightColor);
    m_sceneShader.SetUniform1f("u_Ambient", light.ambient);
}

void Renderer::drawMeshLit(const GpuMesh &gpu, const DirectionalLight &light,
                           const glm::mat4 &mvp)
{
    applyLighting(light);
    drawMesh(gpu, m_sceneShader, mvp);
    m_sceneShader.SetUniform1i("u_Lit", 0);   // unlit is the program's resting state
    m_sceneShader.Unbind();
}

glm::mat4 Renderer::renderScene(const Camera &camera, const Viewport &viewport,
                                const DirectionalLight &light,
                                std::optional<size_t> skyPreset)
{
    setupViewport(viewport);
    glm::mat4 mvp = viewProjection(camera, viewport);
    drawMeshLit(m_terrain, light, mvp);
    // Sky after the terrain (depth confines it to the background) but before
    // the overlays (they draw with depth off and must land on top of it).
    if (skyPreset)
        m_sky.draw(*skyPreset, camera, viewport);
    return mvp;
}

void Renderer::renderGlobalView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light(), sim.lightPreset);

    // The view cone is drawn here rather than in a State's overlay because it
    // is not mode-specific: "where is the player camera pointing" is the same
    // question in every mode, and putting it here keeps five states from
    // repeating the same call. Mode-specific aids (the sight lines) stay in
    // the overlays below, where they belong.
    if (sim.showViewAids)
        drawViewCone(sim.playerView.camera, sim.playerView.viewport.aspect(),
                     viewConeReach(sim), overlay::viewConeColor,
                     overlay::viewConeWidth, mvp);

    if (sim.currentState)
        sim.currentState->renderGlobalOverlay(sim, *this, mvp);
}

void Renderer::renderPlayerView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light(), sim.lightPreset);
    if (sim.currentState)
        sim.currentState->renderPlayerOverlay(sim, *this, mvp);
}

// ── Overlay drawing ───────────────────────────────────────────

void Renderer::drawPath(const std::vector<glm::vec3> &pathPoints, const glm::vec3 &color,
                        const glm::mat4 &mvp)
{
    if (pathPoints.empty()) return;

    m_overlayVerts.clear();
    for (const glm::vec3 &p : pathPoints) {
        m_overlayVerts.push_back(p.x); m_overlayVerts.push_back(p.y); m_overlayVerts.push_back(p.z);
        m_overlayVerts.push_back(color.r); m_overlayVerts.push_back(color.g); m_overlayVerts.push_back(color.b);
    }

    GLCall(glLineWidth(overlay::pathWidth));
    GLCall(glDisable(GL_DEPTH_TEST));
    m_overlayBatch.draw(m_overlayVerts, GL_LINE_STRIP, m_sceneShader, mvp);
    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glLineWidth(1.0f));   // back to the GL default
}

void Renderer::drawWaypoints(const std::vector<Waypoint> &waypoints,
                             const glm::vec3 &cameraPos, const glm::mat4 &mvp)
{
    if (waypoints.empty()) return;

    m_overlayVerts.clear();
    for (const Waypoint &waypoint : waypoints) {
        const glm::vec3 &p = waypoint.position;
        m_overlayVerts.push_back(p.x); m_overlayVerts.push_back(p.y); m_overlayVerts.push_back(p.z);
        // Exact float equality is safe: cameraPos was set by copying a
        // waypoint's position (Camera::applyPose), never recomputed.
        if (p == cameraPos) {
            m_overlayVerts.push_back(0.0f); m_overlayVerts.push_back(1.0f); m_overlayVerts.push_back(0.0f); // green
        } else {
            m_overlayVerts.push_back(1.0f); m_overlayVerts.push_back(0.0f); m_overlayVerts.push_back(0.0f); // red
        }
    }

    GLCall(glPointSize(overlay::waypointMarkerSize));
    m_overlayBatch.draw(m_overlayVerts, GL_POINTS, m_pointShader, mvp);
    GLCall(glPointSize(1.0f));   // back to the GL default
}

// ── Color picking ─────────────────────────────────────────────

// Swap in a pass-specific clear color, restoring the previous one on scope
// exit, so a capture pass can't leak its background into the next visible frame.
struct ScopedClearColor {
    GLfloat prev[4];
    ScopedClearColor(float r, float g, float b, float a)
    {
        GLCall(glGetFloatv(GL_COLOR_CLEAR_VALUE, prev));
        GLCall(glClearColor(r, g, b, a));
    }
    ~ScopedClearColor()
    {
        GLCall(glClearColor(prev[0], prev[1], prev[2], prev[3]));
    }
};

void Renderer::renderPickPass(const Camera &camera, const Viewport &viewport)
{
    ScopedClearColor white(1.0f, 1.0f, 1.0f, 1.0f);
    clear();

    setupViewport(viewport);
    drawMesh(m_terrain, m_pickShader, viewProjection(camera, viewport));
    m_pickShader.Unbind();
}

int Renderer::pickVertex(int mouseX, int mouseY, const View &view)
{
    const Viewport &viewport = view.viewport;
    renderPickPass(view.camera, viewport);

    // glReadPixels uses framebuffer coords (origin bottom-left); the cursor is
    // top-left, hence the (height-1 - mouseY) flip. Assumes full-height
    // viewports (viewport.y == 0), which the split-screen layout guarantees.
    unsigned char px[3];
    GLCall(glReadPixels(mouseX, viewport.height - 1 - mouseY, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, px));

    return decodeVertexId(px, m_terrain.vertexCount);
}

void Renderer::drawMeshFlat(const GpuMesh &gpu, const glm::vec4 &fill, float tintStrength,
                            const glm::mat4 &mvp)
{
    // Local copy: the vendored Shader::SetUniform4f takes a non-const reference.
    glm::vec4 overrideColor = fill;

    m_sceneShader.Bind();
    m_sceneShader.SetUniform1i("u_UseOverride", 1);
    m_sceneShader.SetUniform4f("u_OverrideColor", overrideColor);
    m_sceneShader.SetUniform1f("u_TintStrength", tintStrength);

    drawMesh(gpu, m_sceneShader, mvp);

    m_sceneShader.SetUniform1i("u_UseOverride", 0);
    m_sceneShader.Unbind();
}

void Renderer::drawGhost(const Camera &estimatedCamera, const Viewport &viewport,
                         const glm::vec3 &color, float alpha, float tintStrength)
{
    // Neither write nor test depth: the ghost sits over the real terrain
    // instead of z-fighting with or being occluded by it.
    GLCall(glDepthMask(GL_FALSE));
    GLCall(glDisable(GL_DEPTH_TEST));

    drawMeshFlat(m_terrain, glm::vec4(color, alpha), tintStrength,
                 viewProjection(estimatedCamera, viewport));

    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glDepthMask(GL_TRUE));
}

void Renderer::drawTrackers(const std::vector<Tracker> &trackers, const glm::mat4 &viewProj)
{
    // Flat, unshaded fill: the pixels must land as the exact palette color the
    // blob detector looks for.
    for (const Tracker &tracker : trackers)
        drawMeshFlat(m_sphere, glm::vec4(tracker.color, 1.0f), 1.0f,
                     viewProj * tracker.modelMatrix());
}

// ── Read-back captures ────────────────────────────────────────

FramePixels Renderer::readViewportPixels(const Viewport &viewport)
{
    FramePixels frame;
    frame.width  = viewport.width;
    frame.height = viewport.height;
    frame.rgb.resize((size_t)viewport.width * viewport.height * 3);

    // Force tightly packed rows: the default GL_PACK_ALIGNMENT of 4 pads each
    // row when width * 3 isn't a multiple of 4, which would silently shear
    // every following scanline.
    GLCall(glPixelStorei(GL_PACK_ALIGNMENT, 1));

    GLCall(glReadPixels(viewport.x, viewport.y, viewport.width, viewport.height,
                        GL_RGB, GL_UNSIGNED_BYTE, frame.rgb.data()));

    // GL hands rows bottom-up; flip once here to image convention (row 0 =
    // top). In place, through one row-sized scratch buffer.
    const size_t rowBytes = (size_t)viewport.width * 3;
    std::vector<unsigned char> scratch(rowBytes);
    for (int y = 0; y < viewport.height / 2; y++) {
        unsigned char *top = frame.rgb.data() + (size_t)y * rowBytes;
        unsigned char *bottom = frame.rgb.data()
                              + (size_t)(viewport.height - 1 - y) * rowBytes;
        std::memcpy(scratch.data(), top, rowBytes);
        std::memcpy(top, bottom, rowBytes);
        std::memcpy(bottom, scratch.data(), rowBytes);
    }
    return frame;
}

FramePixels Renderer::captureTrackersFrame(const View &playerView,
                                           const std::vector<Tracker> &trackers,
                                           const DirectionalLight &light)
{
    const Viewport &viewport = playerView.viewport;

    // Black background, no sky pass: above the horizon there is no terrain to
    // separate the trackers from, and no palette color is near black.
    ScopedClearColor black(0.0f, 0.0f, 0.0f, 1.0f);
    clear();

    setupViewport(viewport);
    glm::mat4 viewProj = viewProjection(playerView.camera, viewport);

    drawMeshLit(m_terrain, light, viewProj);
    drawTrackers(trackers, viewProj);

    return readViewportPixels(viewport);
}

FramePixels Renderer::captureSceneFrame(const View &view, const DirectionalLight &light)
{
    clear();
    // no sky: feature matching must not see sky pixels (they have no
    // terrain 3D position and differ per preset).
    renderScene(view.camera, view.viewport, light);
    return readViewportPixels(view.viewport);
}

// RAII home of the capture framebuffer: an FBO with color + depth
// renderbuffers, bound on construction. The destructor rebinds the default
// framebuffer and deletes all three GL objects, so no exit path from a capture
// can leak them. Throwaway per capture by design: captures are
// keypress-triggered or a build pass of a few dozen, never per-frame, so
// creation cost is irrelevant.
struct OffscreenTarget {
    GLuint fbo = 0, colorRb = 0, depthRb = 0;

    OffscreenTarget(int width, int height)
    {
        GLCall(glGenFramebuffers(1, &fbo));
        GLCall(glBindFramebuffer(GL_FRAMEBUFFER, fbo));

        GLCall(glGenRenderbuffers(1, &colorRb));
        GLCall(glBindRenderbuffer(GL_RENDERBUFFER, colorRb));
        GLCall(glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, width, height));
        GLCall(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                         GL_RENDERBUFFER, colorRb));

        // Depth too: occlusion must hide back terrain in the capture exactly
        // as it does on screen.
        GLCall(glGenRenderbuffers(1, &depthRb));
        GLCall(glBindRenderbuffer(GL_RENDERBUFFER, depthRb));
        GLCall(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height));
        GLCall(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                         GL_RENDERBUFFER, depthRb));
        GLCall(glBindRenderbuffer(GL_RENDERBUFFER, 0));
    }

    OffscreenTarget(const OffscreenTarget &) = delete;
    OffscreenTarget &operator=(const OffscreenTarget &) = delete;

    ~OffscreenTarget()
    {
        GLCall(glBindFramebuffer(GL_FRAMEBUFFER, 0));
        GLCall(glDeleteRenderbuffers(1, &depthRb));
        GLCall(glDeleteRenderbuffers(1, &colorRb));
        GLCall(glDeleteFramebuffers(1, &fbo));
    }

    bool complete() const
    {
        return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }
};

FramePixels Renderer::captureSceneFrameAt(int width, int height, const Camera &camera,
                                          const DirectionalLight &light)
{
    OffscreenTarget target(width, height);
    if (!target.complete()) {
        std::cerr << "capture: offscreen framebuffer incomplete at " << width << "x"
                  << height << " -- capture skipped" << std::endl;
        return {};
    }

    const Viewport full{ 0, 0, width, height };
    clear();
    renderScene(camera, full, light);   // no sky, as above
    return readViewportPixels(full);
}

void Renderer::drawPoints(const std::vector<glm::vec3> &positions,
                          const std::vector<glm::vec3> &colors,
                          float size, const glm::mat4 &mvp)
{
    if (positions.empty()) return;

    m_overlayVerts.clear();
    for (size_t i = 0; i < positions.size(); i++) {
        const glm::vec3 &p = positions[i];
        const glm::vec3 &c = colors[i];
        m_overlayVerts.push_back(p.x); m_overlayVerts.push_back(p.y); m_overlayVerts.push_back(p.z);
        m_overlayVerts.push_back(c.r); m_overlayVerts.push_back(c.g); m_overlayVerts.push_back(c.b);
    }

    GLCall(glDisable(GL_DEPTH_TEST));
    GLCall(glPointSize(size));
    m_overlayBatch.draw(m_overlayVerts, GL_POINTS, m_pointShader, mvp);
    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glPointSize(1.0f));   // back to the GL default
}

// ── View aids ─────────────────────────────────────────────────

void Renderer::drawLines(const std::vector<glm::vec3> &segments, const glm::vec3 &color,
                         float width, const glm::mat4 &mvp)
{
    if (segments.size() < 2) return;

    m_overlayVerts.clear();
    for (const glm::vec3 &p : segments) {
        m_overlayVerts.push_back(p.x); m_overlayVerts.push_back(p.y); m_overlayVerts.push_back(p.z);
        m_overlayVerts.push_back(color.r); m_overlayVerts.push_back(color.g); m_overlayVerts.push_back(color.b);
    }

    GLCall(glLineWidth(width));
    GLCall(glDisable(GL_DEPTH_TEST));
    m_overlayBatch.draw(m_overlayVerts, GL_LINES, m_sceneShader, mvp);
    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glLineWidth(1.0f));   // back to the GL default
}

void Renderer::drawViewCone(const Camera &camera, float aspect, float reach,
                            const glm::vec3 &color, float width, const glm::mat4 &mvp)
{
    // The frustum corners are just the viewing rays through the four corners
    // of the image -- the same fractionToRay mapping at uv = (0,0)..(1,1) --
    // so the cone drawn on the map is exactly the wedge the player pane shows.
    const float t = std::tan(glm::radians(camera.fov) * 0.5f);
    const glm::vec2 cornerRays[4] = {
        { -t * aspect, -t },   // top-left
        {  t * aspect, -t },   // top-right
        {  t * aspect,  t },   // bottom-right
        { -t * aspect,  t },   // bottom-left
    };

    glm::vec3 corners[4];
    for (int i = 0; i < 4; i++)
        corners[i] = camera.position + rayDirection(camera, cornerRays[i]) * reach;

    m_overlaySegments.clear();
    for (int i = 0; i < 4; i++) {
        m_overlaySegments.push_back(camera.position); // edge from the eye ...
        m_overlaySegments.push_back(corners[i]);
        m_overlaySegments.push_back(corners[i]);      // ... and the far rectangle
        m_overlaySegments.push_back(corners[(i + 1) % 4]);
    }
    drawLines(m_overlaySegments, color, width, mvp);
}

void Renderer::drawSightLines(const Camera &camera, const std::vector<glm::vec2> &imageRays,
                              float reach, const glm::vec3 &color, float width,
                              const glm::mat4 &mvp)
{
    m_overlaySegments.clear();
    for (const glm::vec2 &ray : imageRays) {
        m_overlaySegments.push_back(camera.position);
        m_overlaySegments.push_back(camera.position + rayDirection(camera, ray) * reach);
    }
    drawLines(m_overlaySegments, color, width, mvp);
}
