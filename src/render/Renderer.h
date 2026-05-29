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

// Window-layout helpers: the two viewports are pure functions of the current
// window size, so we compute them on demand each frame rather than storing them.
Viewport leftHalf(int windowWidth, int windowHeight);
Viewport rightHalf(int windowWidth, int windowHeight);

void setupViewport(const Viewport &viewport);

// The viewport's aspect ratio feeds the projection, so it is an explicit input
// here -- but the camera itself stays untouched (const&).
glm::mat4 computeViewProjection(const Camera &camera, const Viewport &viewport);

void renderTerrain(const TerrainGpu &gpu, Shader &shader, const glm::mat4 &mvp);

void renderGlobalOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp);

void renderPlayerOverlay(const AppState &appState, Shader &shader, const glm::mat4 &mvp);
