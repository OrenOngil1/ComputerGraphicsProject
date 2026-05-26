#include "Renderer.h"

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include <VertexBufferLayout.h>

#include "Overlay.h"

// Builds the terrain's GPU buffers ONCE at startup. After this returns, the
// vertex and index data live in GPU memory and stay there for the rest of the
// program; every frame just issues a single draw call referencing them.
TerrainGpu uploadTerrain(const Mesh &mesh)
{
    // Bake the centering translation into vertex positions instead of doing it
    // every frame with a model matrix. Origin = middle of the terrain.
    const float cx = mesh.width  / 2.0f;
    const float cz = mesh.height / 2.0f;

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
    indices.reserve((mesh.width - 1) * (mesh.height - 1) * 6);
    for (int z = 0; z < mesh.height - 1; z++) {
        for (int x = 0; x < mesh.width - 1; x++) {
            unsigned int i00 = z       * mesh.width + x;
            unsigned int i10 = (z + 1) * mesh.width + x;
            unsigned int i01 = z       * mesh.width + (x + 1);
            unsigned int i11 = (z + 1) * mesh.width + (x + 1);
            indices.push_back(i00); indices.push_back(i10); indices.push_back(i01);
            indices.push_back(i01); indices.push_back(i10); indices.push_back(i11);
        }
    }

    // VertexBuffer / IndexBuffer constructors call glBufferData, which is the
    // actual CPU -> GPU memory transfer. The CPU-side `verts`/`indices` vectors
    // can be freed after this; the GPU has its own copy.
    TerrainGpu gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();

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

// Selects which half of the window this camera draws into and updates the
// camera's stored width/height in case the window has been resized.
void setupCamera(Camera &camera)
{
    glfwGetWindowSize(glfwGetCurrentContext(), &camera.width, &camera.height);

    // TODO(phase3-cleanup): split viewport x-position from camera state; the
    // `camera.x == 0 ? 0 : width` flag-as-coordinate hack should become an
    // explicit left/right enum on the Camera or a separate Viewport struct.
    camera.width /= 2.0f;
    camera.x = camera.x == 0 ? 0 : camera.width;

    glViewport(camera.x, camera.y, camera.width, camera.height);
}

// Build the projection*view matrix on the CPU. The vertex shader will
// multiply it by each vertex position to get the final clip-space coordinate.
// No model matrix yet -- the centering offset is baked into the vertices and
// nothing else moves.
glm::mat4 computeViewProjection(const Camera &camera)
{
    glm::mat4 proj = glm::perspective(
        glm::radians(camera.fov),
        (float)camera.width / (float)camera.height,
        camera.near,
        camera.far);
    glm::mat4 view = glm::lookAt(camera.position, camera.target, camera.up);
    return proj * view;
}

// One draw call: bind the shader, push the MVP uniform, bind the pre-uploaded
// VAO+IBO, and let the GPU rasterize. No vertex data crosses the bus here.
void renderTerrain(const TerrainGpu &gpu, Shader &shader, const glm::mat4 &mvp)
{
    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    gpu.va->Bind();
    gpu.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr));
}

// Overlay for the global (overhead) view: shows the recorded flight path and
// the camera waypoints. Only drawn in RECORD / PLAYBACK modes.
void renderGlobalOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp)
{
    switch (appState.mode) {
        case Mode::NONE:
            return;
        case Mode::RECORD:
        case Mode::PLAYBACK:
            renderPath(appState.pathPoints, shader, mvp);
            renderCameraRecords(appState.cameraRecords, appState.playbackIndex, shader, mvp);
            break;
        default:
            break;
    }
}

// Overlay for the player (first-person) view -- placeholder for a future HUD
// (crosshair, altitude readout, attitude indicator, etc.).
void renderPlayerOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp)
{
    (void)appState; (void)shader; (void)mvp;
}
