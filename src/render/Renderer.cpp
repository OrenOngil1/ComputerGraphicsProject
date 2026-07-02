#include "Renderer.h"

#include <cstring>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>

#include "PickEncoding.h"
#include "../state/State.h"

static void setupViewport(const Viewport &viewport)
{
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);
}

static void drawMesh(const GpuMesh &gpu, Shader &shader, const glm::mat4 &mvp)
{
    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    gpu.va->Bind();
    gpu.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr));
}

// ── Renderer ──────────────────────────────────────────────────

Renderer::Renderer()
    : m_sceneShader("assets/shaders/terrainShader.glsl"),
      m_pickShader("assets/shaders/pickShader.glsl"),
      m_pointShader("assets/shaders/pointShader.glsl")
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
    m_sceneShader.SetUniform1i("u_Lit", 0);
}

void Renderer::drawMeshLit(const GpuMesh &gpu, const glm::vec3 &fill,
                           const DirectionalLight &light, const glm::mat4 &mvp)
{
    // The override path supplies the solid fill; the raised u_Lit shades it.
    applyLighting(light);
    drawMeshFlat(gpu, glm::vec4(fill, 1.0f), 1.0f, mvp);
    m_sceneShader.SetUniform1i("u_Lit", 0);
}

glm::mat4 Renderer::renderScene(const Camera &camera, const Viewport &viewport,
                                const DirectionalLight &light)
{
    setupViewport(viewport);
    glm::mat4 mvp = viewProjection(camera, viewport);
    drawMeshLit(m_terrain, light, mvp);
    return mvp;
}

void Renderer::renderGlobalView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light());
    if (sim.currentState)
        sim.currentState->renderGlobalOverlay(sim, *this, mvp);
}

void Renderer::renderPlayerView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light());
    if (sim.currentState)
        sim.currentState->renderPlayerOverlay(sim, *this, mvp);
}

// ── Overlay drawing ───────────────────────────────────────────

// Upload-and-draw a throwaway vertex batch (xyz + rgb interleaved). Overlay
// contents change every frame, so no long-lived buffers: the destructors free
// the GPU objects on return. Cheap for hundreds of points.
static void drawVertexBatch(const std::vector<float> &verts, GLenum primitive, Shader &shader, const glm::mat4 &mvp)
{
    VertexArray va;
    VertexBuffer vb(verts.data(), verts.size() * sizeof(float));

    VertexBufferLayout layout;
    layout.Push<float>(3);  // position
    layout.Push<float>(3);  // color
    va.AddBuffer(vb, layout);

    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    va.Bind();
    GLCall(glDrawArrays(primitive, 0, (GLsizei)(verts.size() / 6)));   // 6 floats per vertex
}

void Renderer::drawPath(const std::vector<glm::vec3> &pathPoints, const glm::vec3 &color,
                        const glm::mat4 &mvp)
{
    if (pathPoints.empty()) return;

    std::vector<float> verts;
    verts.reserve(pathPoints.size() * 6);
    for (const glm::vec3 &p : pathPoints) {
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(color.r); verts.push_back(color.g); verts.push_back(color.b);
    }

    GLCall(glLineWidth(3.0f));
    GLCall(glDisable(GL_DEPTH_TEST));
    drawVertexBatch(verts, GL_LINE_STRIP, m_sceneShader, mvp);
    GLCall(glEnable(GL_DEPTH_TEST));
}

void Renderer::drawWaypoints(const std::vector<Waypoint> &waypoints,
                             const glm::vec3 &cameraPos, const glm::mat4 &mvp)
{
    if (waypoints.empty()) return;

    std::vector<float> verts;
    verts.reserve(waypoints.size() * 6);
    for (const Waypoint &waypoint : waypoints) {
        const glm::vec3 &p = waypoint.position;
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        // Exact float equality is safe: cameraPos was set by copying a
        // waypoint's position (Camera::applyPose), never recomputed.
        if (p == cameraPos) {
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f); // green
        } else {
            verts.push_back(1.0f); verts.push_back(0.0f); verts.push_back(0.0f); // red
        }
    }

    GLCall(glPointSize(5.0f));
    drawVertexBatch(verts, GL_POINTS, m_pointShader, mvp);
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
}

int Renderer::pickVertex(int mouseX, int mouseY, const View &playerView)
{
    const Viewport &viewport = playerView.viewport;
    renderPickPass(playerView.camera, viewport);

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

void Renderer::drawTrackersLit(const std::vector<Tracker> &trackers,
                               const DirectionalLight &light, const glm::mat4 &viewProj)
{
    for (const Tracker &tracker : trackers)
        drawMeshLit(m_sphere, tracker.color, light, viewProj * tracker.modelMatrix());
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
                                           const std::vector<Tracker> &trackers)
{
    const Viewport &viewport = playerView.viewport;

    ScopedClearColor black(0.0f, 0.0f, 0.0f, 1.0f);
    clear();

    setupViewport(viewport);
    glm::mat4 viewProj = viewProjection(playerView.camera, viewport);

    // Terrain flat black, full tint: it still writes depth (occlusion as in
    // the visible frame) but contributes no color that could collide with a
    // tracker's.
    drawMeshFlat(m_terrain, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), 1.0f, viewProj);
    drawTrackers(trackers, viewProj);

    return readViewportPixels(viewport);
}

FramePixels Renderer::captureSceneFrame(const View &view, const DirectionalLight &light)
{
    clear();
    renderScene(view.camera, view.viewport, light);
    return readViewportPixels(view.viewport);
}

void Renderer::drawPoints(const std::vector<glm::vec3> &positions,
                          const std::vector<glm::vec3> &colors,
                          float size, const glm::mat4 &mvp)
{
    if (positions.empty()) return;

    std::vector<float> verts;
    verts.reserve(positions.size() * 6);
    for (size_t i = 0; i < positions.size(); i++) {
        const glm::vec3 &p = positions[i];
        const glm::vec3 &c = colors[i];
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(c.r); verts.push_back(c.g); verts.push_back(c.b);
    }

    GLCall(glDisable(GL_DEPTH_TEST));
    GLCall(glPointSize(size));
    drawVertexBatch(verts, GL_POINTS, m_pointShader, mvp);
    GLCall(glEnable(GL_DEPTH_TEST));
}
