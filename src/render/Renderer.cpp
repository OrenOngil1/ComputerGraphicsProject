#include "Renderer.h"

#include <cmath>
#include <cstring>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>

#include "../state/State.h"

// Upload interleaved vertices + triangle indices and record the layout in a
// VAO. attribSizes lists the float count of each attribute in push order,
// which maps to shader attribute locations: 0 = position, 1 = color,
// 2 = normal where present.
static GpuMesh uploadBuffers(const std::vector<float> &verts,
                             const std::vector<unsigned int> &indices,
                             const std::vector<int> &attribSizes)
{
    int floatsPerVertex = 0;
    for (int size : attribSizes)
        floatsPerVertex += size;

    GpuMesh gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();
    gpu.vertexCount = verts.size() / floatsPerVertex;

    VertexBufferLayout layout;
    for (int size : attribSizes)
        layout.Push<float>(size);
    gpu.va->AddBuffer(*gpu.vb, layout);

    // Unbind so later binds can't accidentally modify these objects.
    gpu.va->Unbind();
    gpu.vb->Unbind();
    gpu.ib->Unbind();
    return gpu;
}

// Build the terrain's GPU buffers: interleaved position/color/normal, with the
// centering translation baked into the positions (Mesh::center is the shared
// definition every vertex->world consumer uses). Normals pass through
// untransformed -- translation doesn't change them.
static GpuMesh uploadTerrain(const Mesh &mesh)
{
    const glm::vec3 center = mesh.center();

    std::vector<float> verts;
    verts.reserve(mesh.vertices.size() * 9);
    for (const Vertex &v : mesh.vertices) {
        verts.push_back(v.position.x - center.x);
        verts.push_back(v.position.y);
        verts.push_back(v.position.z - center.z);
        verts.push_back(v.color.r);
        verts.push_back(v.color.g);
        verts.push_back(v.color.b);
        verts.push_back(v.normal.x);
        verts.push_back(v.normal.y);
        verts.push_back(v.normal.z);
    }

    // Each grid cell is two triangles sharing a diagonal:
    //
    //   i00 --- i01        first  triangle: i00 -> i10 -> i01
    //    |  \    |         second triangle: i01 -> i10 -> i11
    //   i10 --- i11
    std::vector<unsigned int> indices;
    indices.reserve((mesh.cols - 1) * (mesh.rows - 1) * 6);
    for (int z = 0; z < mesh.rows - 1; z++) {
        for (int x = 0; x < mesh.cols - 1; x++) {
            unsigned int i00 = z       * mesh.cols + x;
            unsigned int i10 = (z + 1) * mesh.cols + x;
            unsigned int i01 = z       * mesh.cols + (x + 1);
            unsigned int i11 = (z + 1) * mesh.cols + (x + 1);
            indices.push_back(i00); indices.push_back(i10); indices.push_back(i01);
            indices.push_back(i01); indices.push_back(i10); indices.push_back(i11);
        }
    }

    return uploadBuffers(verts, indices, { 3, 3, 3 });   // position, color, normal
}

// Build the unit-radius UV sphere all trackers share; each draw scales and
// translates it into place. The white vertex color satisfies the 6-float
// layout -- tracker draws always override it. No normal attribute: trackers
// draw flat, never lit.
static GpuMesh buildSphereMesh(int stacks, int sectors)
{
    std::vector<float> verts;
    verts.reserve((stacks + 1) * (sectors + 1) * 6);
    for (int i = 0; i <= stacks; i++) {
        // phi sweeps pole to pole, theta around the equator; the seam column
        // (j == sectors) duplicates j == 0 so the index grid can wrap simply.
        float phi = glm::pi<float>() * i / stacks;
        for (int j = 0; j <= sectors; j++) {
            float theta = 2.0f * glm::pi<float>() * j / sectors;
            verts.push_back(std::sin(phi) * std::cos(theta));
            verts.push_back(std::cos(phi));
            verts.push_back(std::sin(phi) * std::sin(theta));
            verts.push_back(1.0f); verts.push_back(1.0f); verts.push_back(1.0f);
        }
    }

    // Two triangles per lat-long cell, same diagonal split as the terrain grid.
    std::vector<unsigned int> indices;
    indices.reserve(stacks * sectors * 6);
    for (int i = 0; i < stacks; i++) {
        for (int j = 0; j < sectors; j++) {
            unsigned int i00 = i       * (sectors + 1) + j;
            unsigned int i10 = (i + 1) * (sectors + 1) + j;
            indices.push_back(i00); indices.push_back(i10); indices.push_back(i00 + 1);
            indices.push_back(i00 + 1); indices.push_back(i10); indices.push_back(i10 + 1);
        }
    }

    return uploadBuffers(verts, indices, { 3, 3 });   // position, color
}

static void setupViewport(const Viewport &viewport)
{
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);
}

// Projection * view. The aspect comes from the viewport, not the camera; no
// model matrix -- the centering offset is baked into the vertices.
static glm::mat4 computeViewProjection(const Camera &camera, const Viewport &viewport)
{
    glm::mat4 proj = glm::perspective(
        glm::radians(camera.fov),
        (float)viewport.width / (float)viewport.height,
        camera.near,
        camera.far
    );
    glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    return proj * view;
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

glm::mat4 Renderer::renderScene(const Camera &camera, const Viewport &viewport,
                                const DirectionalLight &light)
{
    setupViewport(viewport);
    glm::mat4 mvp = computeViewProjection(camera, viewport);

    // Raise u_Lit for the terrain draw only, then drop it: every other draw
    // through this program (paths, ghost, trackers, capture passes) must keep
    // its exact unshaded colors -- the read-back pipelines depend on that.
    m_sceneShader.Bind();
    glm::vec4 lightDir(light.direction, 0.0f);
    glm::vec4 lightColor(light.color, 1.0f);
    m_sceneShader.SetUniform1i("u_Lit", 1);
    m_sceneShader.SetUniform4f("u_LightDir", lightDir);
    m_sceneShader.SetUniform4f("u_LightColor", lightColor);
    m_sceneShader.SetUniform1f("u_Ambient", light.ambient);

    drawMesh(m_terrain, m_sceneShader, mvp);

    m_sceneShader.SetUniform1i("u_Lit", 0);
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

// Decode a pick-pass pixel back into a vertex id: the pick shader packs the id
// little-endian into RGB. Out of range -- the white background, or a pixel not
// from the pick pass -- decodes to -1 (a miss). RGB8 caps ids at 2^24, far
// beyond any terrain this renders interactively.
static int decodeVertexId(const unsigned char *px, size_t vertexCount)
{
    unsigned int id = px[0] | (px[1] << 8) | (px[2] << 16);
    return id < vertexCount ? (int)id : -1;
}

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
    drawMesh(m_terrain, m_pickShader, computeViewProjection(camera, viewport));
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
                 computeViewProjection(estimatedCamera, viewport));

    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glDepthMask(GL_TRUE));
}

void Renderer::drawTrackers(const std::vector<Tracker> &trackers, const glm::mat4 &viewProj)
{
    // Flat, unshaded fill: the pixels must land as the exact palette color the
    // blob detector looks for.
    for (const Tracker &tracker : trackers) {
        glm::mat4 model = glm::scale(glm::translate(glm::mat4(1.0f), tracker.center),
                                     glm::vec3(tracker.radius));
        drawMeshFlat(m_sphere, glm::vec4(tracker.color, 1.0f), 1.0f, viewProj * model);
    }
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
    glm::mat4 viewProj = computeViewProjection(playerView.camera, viewport);

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
