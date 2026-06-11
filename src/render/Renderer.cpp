#include "Renderer.h"

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include <VertexArray.h>
#include <VertexBuffer.h>
#include <VertexBufferLayout.h>

#include "../state/State.h"

// Builds the terrain's GPU buffers from a mesh. After this returns, the vertex and
// index data live in GPU memory; every frame just issues a single draw call
// referencing them. Called by loadTerrain -- once per terrain, reused across frames.
GpuMesh uploadTerrain(const Mesh &mesh)
{
    // Bake the centering translation into vertex positions instead of doing it
    // every frame with a model matrix. Origin = middle of the terrain.
    const float cx = mesh.cols / 2.0f;
    const float cz = mesh.rows / 2.0f;

    // Interleaved layout: [x,y,z, r,g,b, x,y,z, r,g,b, ...] -- 6 floats per vertex.
    // The shader reads attribute 0 (position) from offset 0 and attribute 1
    // (color) from offset 12, with stride 24.
    std::vector<float> verts;
    verts.reserve(mesh.vertices.size() * 6);
    for (const Vertex &v : mesh.vertices) {
        verts.push_back(v.position.x - cx);
        verts.push_back(v.position.y);
        verts.push_back(v.position.z - cz);
        verts.push_back(v.color.r);
        verts.push_back(v.color.g);
        verts.push_back(v.color.b);
    }

    // Each cell of the height-map grid is two triangles sharing a diagonal:
    //
    //   i00 --- i01        first  triangle: i00 -> i10 -> i01
    //    |  \    |         second triangle: i01 -> i10 -> i11
    //   i10 --- i11
    //
    // Using indices lets a vertex be reused by up to 6 surrounding triangles
    // instead of being duplicated, which keeps the VBO small.
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

    // VertexBuffer / IndexBuffer constructors call glBufferData, which is the
    // actual CPU -> GPU memory transfer. The CPU-side `verts`/`indices` vectors
    // can be freed after this; the GPU has its own copy.
    GpuMesh gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();
    gpu.vertexCount = mesh.vertices.size();

    // The VAO records "how to read the VBO's bytes" -- not the bytes themselves.
    // Push order maps to shader attribute locations: 0 = position, 1 = color.
    VertexBufferLayout layout;
    layout.Push<float>(3);  // position
    layout.Push<float>(3);  // color
    gpu.va->AddBuffer(*gpu.vb, layout);

    // Defensive unbind so nothing else accidentally modifies these objects.
    gpu.va->Unbind();
    gpu.vb->Unbind();
    gpu.ib->Unbind();
    return gpu;
}

// Tell GL which rectangle of the framebuffer subsequent draws land in.
void setupViewport(const Viewport &viewport)
{
    glViewport(viewport.x, viewport.y, viewport.width, viewport.height);
}

// Build the projection*view matrix on the CPU. The vertex shader will
// multiply it by each vertex position to get the final clip-space coordinate.
// The aspect ratio comes from the viewport, not the camera. No model matrix
// yet -- the centering offset is baked into the vertices and nothing else moves.
glm::mat4 computeViewProjection(const Camera &camera, const Viewport &viewport)
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

// One draw call: bind the shader, push the MVP uniform, bind the pre-uploaded
// VAO+IBO, and let the GPU rasterize. No vertex data crosses the bus here.
void drawMesh(const GpuMesh &gpu, Shader &shader, const glm::mat4 &mvp)
{
    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    gpu.va->Bind();
    gpu.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr));
}

// ── Renderer ──────────────────────────────────────────────────
// A thin owner over the free helpers above. The constructor compiles the shader
// programs only (no terrain yet -- that arrives via loadTerrain); the renderView
// methods sequence the same four steps the duplicated viewport blocks in main()
// used to repeat by hand.

Renderer::Renderer()
    : m_sceneShader("assets/shaders/terrainShader.glsl"),
      m_pickShader("assets/shaders/pickShader.glsl"),
      m_pointShader("assets/shaders/pointShader.glsl")
{
    // m_terrain is left default-constructed (empty buffers); loadTerrain uploads
    // the first mesh. The compiled shaders above only need a live GL context.
}

void Renderer::clear() const
{
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
}

void Renderer::loadTerrain(const Mesh &terrain)
{
    // Move-assign the new buffers in; the old TerrainGpu's unique_ptrs free the
    // previous VAO/VBO/IBO. The compiled shader is untouched -- no recompile.
    m_terrain = uploadTerrain(terrain);
}

// Shared scene pass: set the viewport, build the MVP, draw the terrain. Returns
// the MVP so the view methods can hand it to the active mode's overlay.
glm::mat4 Renderer::renderScene(const Camera &camera, const Viewport &viewport)
{
    setupViewport(viewport);
    glm::mat4 mvp = computeViewProjection(camera, viewport);
    drawMesh(m_terrain, m_sceneShader, mvp);
    return mvp;
}

// The overlay is the active mode's responsibility: Renderer no longer knows about
// Mode, it just asks the State to decorate the view it drew.
void Renderer::renderGlobalView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport);
    if (sim.currentState)
        sim.currentState->renderGlobalOverlay(sim, *this, mvp);
}

void Renderer::renderPlayerView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport);
    if (sim.currentState)
        sim.currentState->renderPlayerOverlay(sim, *this, mvp);
}

// ── Overlay drawing ───────────────────────────────────────────
// Moved in from the old Overlay.{h,cpp}: the mode draws through these methods,
// which supply m_sceneShader themselves, so the shader never leaves the Renderer.

// Overlays differ from terrain in one important way: their contents CHANGE
// every frame (the recorded path grows, the highlighted waypoint moves). So instead
// of building a long-lived VAO/VBO like the terrain does, we build a throwaway
// pair each call -- upload the latest data, draw it, and let the destructors
// free the GPU buffers when the function returns. Cheap for hundreds of points.
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
    // verts.size() / 6 because each vertex is 6 floats (xyz + rgb).
    GLCall(glDrawArrays(primitive, 0, (GLsizei)(verts.size() / 6)));
}

// Flight path: a line strip connecting all positions in order, in one caller-
// chosen color -- the pose-comparison modes draw two paths (true vs computed)
// side by side, so the color identifies which is which.
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

// Camera waypoints: green for the waypoint the player camera is on (position
// match), red for the rest. The highlight tracks the camera, not any cursor.
void Renderer::drawWaypoints(const std::vector<Waypoint> &waypoints,
                             const glm::vec3 &cameraPos, const glm::mat4 &mvp)
{
    if (waypoints.empty()) return;

    std::vector<float> verts;
    verts.reserve(waypoints.size() * 6);
    for (const Waypoint &waypoint : waypoints) {
        const glm::vec3 &p = waypoint.position;
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        if (p == cameraPos) {
            verts.push_back(0.0f); verts.push_back(1.0f); verts.push_back(0.0f); // green
        } else {
            verts.push_back(1.0f); verts.push_back(0.0f); verts.push_back(0.0f); // red
        }
    }

    GLCall(glPointSize(5.0f));
    drawVertexBatch(verts, GL_POINTS, m_pointShader, mvp);
}

// ── PICK mode (Mode 2) ────────────────────────────────────────

// Render the terrain with the flat per-vertex-id shader and read back the pixel
// under the cursor to learn which vertex was clicked. The image goes to the back
// buffer and is never swapped, so the next loop iteration paints over it.
int Renderer::pickVertex(int mouseX, int mouseY, const View &playerView)
{
    const Viewport &viewport = playerView.viewport;

    // Save the persistent clear color so the visible frame is unaffected.
    GLfloat prevClear[4];
    GLCall(glGetFloatv(GL_COLOR_CLEAR_VALUE, prevClear));

    // White background: a click hitting no geometry decodes to 0xFFFFFF (a miss).
    GLCall(glClearColor(1.0f, 1.0f, 1.0f, 1.0f));
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    setupViewport(viewport);
    drawMesh(m_terrain, m_pickShader, computeViewProjection(playerView.camera, viewport));

    // glReadPixels uses framebuffer coords (origin bottom-left); the cursor is
    // top-left. viewport.y is 0 and viewport.height is the full framebuffer height,
    // so the flip is (height-1 - mouseY); mouseX is already an absolute fb x.
    unsigned char px[3];
    GLCall(glReadPixels(mouseX, viewport.height - 1 - mouseY, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, px));

    // Restore the clear color for the normal render path.
    GLCall(glClearColor(prevClear[0], prevClear[1], prevClear[2], prevClear[3]));

    unsigned int id = px[0] | (px[1] << 8) | (px[2] << 16);
    return id < m_terrain.vertexCount ? (int)id : -1;
}

// Redraw the terrain from an estimated pose in one translucent color, overlaid on
// the current view -- the PnP "ghost" the user compares against the real terrain.
void Renderer::drawGhost(const Camera &estimatedCamera, const Viewport &viewport,
                         const glm::vec3 &color, float alpha, float tintStrength)
{
    // Set the override uniforms while m_sceneShader is bound; they persist through
    // drawMesh's (redundant, same-program) re-bind below.
    m_sceneShader.Bind();
    m_sceneShader.SetUniform1i("u_UseOverride", 1);
    glm::vec4 overrideColor(color, alpha);
    m_sceneShader.SetUniform4f("u_OverrideColor", overrideColor);
    m_sceneShader.SetUniform1f("u_TintStrength", tintStrength);

    // See-through overlay: don't write or test depth, so the ghost sits over the
    // real terrain instead of z-fighting with or being occluded by it.
    GLCall(glDepthMask(GL_FALSE));
    GLCall(glDisable(GL_DEPTH_TEST));

    drawMesh(m_terrain, m_sceneShader, computeViewProjection(estimatedCamera, viewport));

    GLCall(glEnable(GL_DEPTH_TEST));
    GLCall(glDepthMask(GL_TRUE));
    m_sceneShader.SetUniform1i("u_UseOverride", 0);   // reset for subsequent draws
}

// Colored point markers (picked correspondence points, the estimated camera, ...),
// one color per position. Drawn on top (depth test off) so markers on the terrain
// surface stay visible.
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
