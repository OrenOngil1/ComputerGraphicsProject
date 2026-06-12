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

// Upload interleaved vertices + triangle indices to the GPU and record the
// layout in a VAO. The shared tail of every long-lived mesh build (terrain,
// tracker sphere). attribSizes lists the float count of each attribute in
// push order (push order maps to shader attribute locations: 0 = position,
// 1 = color, 2 = normal where present). The CPU-side vectors can be freed
// after this; the GPU has its own copy.
static GpuMesh uploadBuffers(const std::vector<float> &verts,
                             const std::vector<unsigned int> &indices,
                             const std::vector<int> &attribSizes)
{
    int floatsPerVertex = 0;
    for (int size : attribSizes)
        floatsPerVertex += size;

    // VertexBuffer / IndexBuffer constructors call glBufferData, which is the
    // actual CPU -> GPU memory transfer.
    GpuMesh gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();
    gpu.vertexCount = verts.size() / floatsPerVertex;

    // The VAO records "how to read the VBO's bytes" -- not the bytes themselves.
    VertexBufferLayout layout;
    for (int size : attribSizes)
        layout.Push<float>(size);
    gpu.va->AddBuffer(*gpu.vb, layout);

    // Defensive unbind so nothing else accidentally modifies these objects.
    gpu.va->Unbind();
    gpu.vb->Unbind();
    gpu.ib->Unbind();
    return gpu;
}

// Builds the terrain's GPU buffers from a mesh. After this returns, the vertex and
// index data live in GPU memory; every frame just issues a single draw call
// referencing them. Called by loadTerrain -- once per terrain, reused across frames.
GpuMesh uploadTerrain(const Mesh &mesh)
{
    // Bake the centering translation into vertex positions instead of doing it
    // every frame with a model matrix. Origin = middle of the terrain.
    const float cx = mesh.cols / 2.0f;
    const float cz = mesh.rows / 2.0f;

    // Interleaved layout: [x,y,z, r,g,b, nx,ny,nz, ...] -- 9 floats per vertex.
    // The shader reads attribute 0 (position), 1 (color), 2 (normal). Normals
    // pass through untransformed: the only "model transform" is the centering
    // translation baked in here, and translation doesn't change normals.
    std::vector<float> verts;
    verts.reserve(mesh.vertices.size() * 9);
    for (const Vertex &v : mesh.vertices) {
        verts.push_back(v.position.x - cx);
        verts.push_back(v.position.y);
        verts.push_back(v.position.z - cz);
        verts.push_back(v.color.r);
        verts.push_back(v.color.g);
        verts.push_back(v.color.b);
        verts.push_back(v.normal.x);
        verts.push_back(v.normal.y);
        verts.push_back(v.normal.z);
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

    return uploadBuffers(verts, indices, { 3, 3, 3 });   // position, color, normal
}

// Build the shared unit-radius UV sphere the trackers are drawn with. Built
// once (the Renderer constructor); each tracker draws it through a model
// matrix (translate + scale) and a flat color, so adding trackers costs no
// GPU memory. The dummy white vertex color satisfies the shared 6-float
// layout -- tracker draws always override it with the tracker's color.
static GpuMesh buildSphereMesh(int stacks, int sectors)
{
    std::vector<float> verts;
    verts.reserve((stacks + 1) * (sectors + 1) * 6);
    for (int i = 0; i <= stacks; i++) {
        // phi sweeps pole to pole, theta around the equator. The seam column
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

    // No normal attribute: trackers draw flat through the override path, which
    // never lights, so the sphere stays at the lean 6-float layout.
    return uploadBuffers(verts, indices, { 3, 3 });   // position, color
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
    // the first mesh. The shared tracker sphere is terrain-independent, so it is
    // built here, once. Both only need the live GL context the Window provides.
    m_sphere = buildSphereMesh(16, 24);
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

// Shared scene pass: set the viewport, build the MVP, draw the terrain lit.
// Returns the MVP so the view methods can hand it to the active mode's overlay.
glm::mat4 Renderer::renderScene(const Camera &camera, const Viewport &viewport,
                                const DirectionalLight &light)
{
    setupViewport(viewport);
    glm::mat4 mvp = computeViewProjection(camera, viewport);

    // Lighting applies to the lit terrain pass ONLY: raise u_Lit while the
    // scene shader is bound, then drop it after the draw, so every other draw
    // through this program (paths, ghost, trackers, capture passes) keeps its
    // exact unshaded colors -- the read-back pipelines depend on that.
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

// The overlay is the active mode's responsibility: Renderer no longer knows about
// Mode, it just asks the State to decorate the view it drew.
void Renderer::renderGlobalView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light);
    if (sim.currentState)
        sim.currentState->renderGlobalOverlay(sim, *this, mvp);
}

void Renderer::renderPlayerView(const View &view, const Simulation &sim)
{
    glm::mat4 mvp = renderScene(view.camera, view.viewport, sim.light);
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

// Decode one pick-pass pixel back into a vertex id: the pick shader packs the
// id little-endian into RGB. Anything out of range -- the white background, or
// a pixel that isn't from the pick pass at all -- maps to -1 (a miss).
static int decodeVertexId(const unsigned char *px, size_t vertexCount)
{
    unsigned int id = px[0] | (px[1] << 8) | (px[2] << 16);
    return id < vertexCount ? (int)id : -1;
}

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

    return decodeVertexId(px, m_terrain.vertexCount);
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

// Draw one tracker: the shared unit sphere through a model matrix (translate +
// scale), filled flat with the tracker's color via the scene shader's override
// path with full tint -- deliberately flat/unshaded, so the pixels land in the
// framebuffer as the exact palette color the blob detector will look for.
// Depth testing stays ON (unlike the overlay helpers): occluded trackers are
// hidden, in the visible frame and the capture read-back alike.
void Renderer::drawTracker(const Tracker &tracker, const glm::mat4 &viewProj)
{
    m_sceneShader.Bind();
    m_sceneShader.SetUniform1i("u_UseOverride", 1);
    glm::vec4 fill(tracker.color, 1.0f);
    m_sceneShader.SetUniform4f("u_OverrideColor", fill);
    m_sceneShader.SetUniform1f("u_TintStrength", 1.0f);   // flat fill, ignore vertex color

    glm::mat4 model = glm::scale(glm::translate(glm::mat4(1.0f), tracker.center),
                                 glm::vec3(tracker.radius));
    drawMesh(m_sphere, m_sceneShader, viewProj * model);

    m_sceneShader.SetUniform1i("u_UseOverride", 0);   // reset for subsequent draws
}

// Pull the viewport's back-buffer pixels into image-convention RGB.
FramePixels Renderer::readViewportPixels(const Viewport &viewport)
{
    FramePixels frame;
    frame.width  = viewport.width;
    frame.height = viewport.height;
    frame.rgb.resize((size_t)viewport.width * viewport.height * 3);

    // Force tightly packed rows: the default GL_PACK_ALIGNMENT of 4 pads each
    // row when width * 3 isn't a multiple of 4, which would silently shear
    // every following scanline. (pickVertex reads one pixel, so it never hits
    // this; full frames do.)
    GLCall(glPixelStorei(GL_PACK_ALIGNMENT, 1));

    std::vector<unsigned char> raw(frame.rgb.size());
    GLCall(glReadPixels(viewport.x, viewport.y, viewport.width, viewport.height,
                        GL_RGB, GL_UNSIGNED_BYTE, raw.data()));

    // GL hands rows bottom-up; flip once here to image convention (row 0 = top)
    // so every consumer indexes pixels the way cursor coordinates work.
    const size_t rowBytes = (size_t)viewport.width * 3;
    for (int y = 0; y < viewport.height; y++)
        std::memcpy(frame.rgb.data() + (size_t)y * rowBytes,
                    raw.data() + (size_t)(viewport.height - 1 - y) * rowBytes,
                    rowBytes);
    return frame;
}

// Render the tracker-detection frame and read it back: flat-black terrain (it
// still writes depth, so it occludes exactly as in the visible frame) plus the
// trackers in their flat palette colors -- every non-black pixel is tracker.
FramePixels Renderer::captureTrackersFrame(const View &playerView,
                                           const std::vector<Tracker> &trackers)
{
    const Viewport &viewport = playerView.viewport;

    // Save the persistent clear color so the visible frame is unaffected.
    GLfloat prevClear[4];
    GLCall(glGetFloatv(GL_COLOR_CLEAR_VALUE, prevClear));

    GLCall(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    setupViewport(viewport);
    glm::mat4 viewProj = computeViewProjection(playerView.camera, viewport);

    // Terrain through the override path: flat black, full tint -- shape and
    // depth only, no color that could collide with a tracker's.
    m_sceneShader.Bind();
    m_sceneShader.SetUniform1i("u_UseOverride", 1);
    glm::vec4 black(0.0f, 0.0f, 0.0f, 1.0f);
    m_sceneShader.SetUniform4f("u_OverrideColor", black);
    m_sceneShader.SetUniform1f("u_TintStrength", 1.0f);
    drawMesh(m_terrain, m_sceneShader, viewProj);
    m_sceneShader.SetUniform1i("u_UseOverride", 0);

    for (const Tracker &tracker : trackers)
        drawTracker(tracker, viewProj);

    FramePixels frame = readViewportPixels(viewport);

    // Restore the clear color for the normal render path.
    GLCall(glClearColor(prevClear[0], prevClear[1], prevClear[2], prevClear[3]));
    return frame;
}

// ── FEATURE MATCH captures (Mode 4) ───────────────────────────

// Render the lit scene as the player view would show it (no overlays) and read
// it back -- the frame feature detection sees. Back buffer only, never swapped.
FramePixels Renderer::captureSceneFrame(const View &view, const DirectionalLight &light)
{
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    renderScene(view.camera, view.viewport, light);
    return readViewportPixels(view.viewport);
}

// Render the pick pass over the whole viewport and decode every pixel. With
// captureSceneFrame of the same View, pixel (x, y) in one is pixel (x, y) in
// the other -- that alignment is what anchors a 2D keypoint to its 3D point.
std::vector<int> Renderer::captureVertexIdFrame(const View &view)
{
    const Viewport &viewport = view.viewport;

    // Save the persistent clear color so the visible frame is unaffected.
    GLfloat prevClear[4];
    GLCall(glGetFloatv(GL_COLOR_CLEAR_VALUE, prevClear));

    // White background, as in pickVertex: a no-terrain pixel decodes to a miss.
    GLCall(glClearColor(1.0f, 1.0f, 1.0f, 1.0f));
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    setupViewport(viewport);
    drawMesh(m_terrain, m_pickShader, computeViewProjection(view.camera, viewport));

    FramePixels frame = readViewportPixels(viewport);
    GLCall(glClearColor(prevClear[0], prevClear[1], prevClear[2], prevClear[3]));

    std::vector<int> ids((size_t)frame.width * frame.height);
    for (size_t i = 0; i < ids.size(); i++)
        ids[i] = decodeVertexId(&frame.rgb[i * 3], m_terrain.vertexCount);
    return ids;
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
