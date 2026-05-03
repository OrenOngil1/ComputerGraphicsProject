#pragma once

#include <iostream>
#include <memory>

#include "../core/AppState.h"

void renderGlobalOverlay(const AppState &appState);

void renderPlayerOverlay(const AppState &appState);

void setupCamera(Camera &camera);

void renderTerrain(const Mesh &mesh, const std::vector<glm::vec3> *colorCodes = nullptr, const glm::vec3 *overrideColor = nullptr, float alpha = 1.0f);

void renderGhostTerrain(const Mesh &mesh, const glm::vec3 &color, float alpha);