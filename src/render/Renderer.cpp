#include "Renderer.h"

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include <VertexBufferLayout.h>

#include "Overlay.h"

TerrainGpu uploadTerrain(const Mesh &mesh)
{
    // Interleaved (pos.xyz, color.rgb) with terrain centered at origin
    const float cx = mesh.width  / 2.0f;
    const float cz = mesh.height / 2.0f;

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

    // Index two triangles per quad: (z,x)-(z+1,x)-(z,x+1) and (z,x+1)-(z+1,x)-(z+1,x+1)
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

    TerrainGpu gpu;
    gpu.va = std::make_unique<VertexArray>();
    gpu.vb = std::make_unique<VertexBuffer>(verts.data(), verts.size() * sizeof(float));
    gpu.ib = std::make_unique<IndexBuffer>(indices.data(), indices.size() * sizeof(unsigned int));
    gpu.indexCount = (unsigned int)indices.size();

    VertexBufferLayout layout;
    layout.Push<float>(3);  // position
    layout.Push<float>(3);  // color
    gpu.va->AddBuffer(*gpu.vb, layout);

    gpu.va->Unbind();
    gpu.vb->Unbind();
    gpu.ib->Unbind();
    return gpu;
}

void setupCamera(Camera &camera)
{
    // Update viewport size in case the window was resized
    glfwGetWindowSize(glfwGetCurrentContext(), &camera.width, &camera.height);

    // TODO(phase3-cleanup): split viewport x-position from camera state; the
    // `camera.x == 0 ? 0 : width` flag-as-coordinate hack should become an
    // explicit left/right enum on the Camera or a separate Viewport struct.
    camera.width /= 2.0f;
    camera.x = camera.x == 0 ? 0 : camera.width;

    glViewport(camera.x, camera.y, camera.width, camera.height);
}

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

void renderTerrain(const TerrainGpu &gpu, Shader &shader, const glm::mat4 &mvp)
{
    shader.Bind();
    shader.SetUniformMat4f("u_MVP", mvp);
    gpu.va->Bind();
    gpu.ib->Bind();
    GLCall(glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr));
}

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

void renderPlayerOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp)
{
    (void)appState; (void)shader; (void)mvp;
    // No player overlay for now
}
