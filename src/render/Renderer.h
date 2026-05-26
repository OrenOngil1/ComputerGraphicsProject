#pragma once

#include <memory>
#include <string>

#include <Shader.h>
#include <VertexArray.h>
#include <VertexBuffer.h>
#include <IndexBuffer.h>

#include "../core/AppState.h"

struct TerrainGpu {
    std::unique_ptr<VertexArray> va;
    std::unique_ptr<VertexBuffer> vb;
    std::unique_ptr<IndexBuffer> ib;
    unsigned int indexCount = 0;
};

TerrainGpu uploadTerrain(const Mesh &mesh);

void setupCamera(Camera &camera);

glm::mat4 computeViewProjection(const Camera &camera);

void renderTerrain(const TerrainGpu &gpu, Shader &shader, const glm::mat4 &mvp);

void renderGlobalOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp);

void renderPlayerOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp);
